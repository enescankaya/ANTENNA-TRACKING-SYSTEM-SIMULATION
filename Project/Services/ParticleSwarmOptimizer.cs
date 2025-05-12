using System;
using System.Collections.Generic;
using System.Linq;

namespace Project.Services
{
    /// <summary>
    /// Enhanced PSO implementation for antenna tracking with improved convergence and dynamic adaptation
    /// </summary>
    public class ParticleSwarmOptimizer
    {
        private readonly Random random = new Random();
        private readonly List<Particle2D> particles;
        private double bestHorizontalAngle;
        private double bestVerticalAngle;
        private double bestFitness;

        // PSO parameters optimized for antenna tracking
        private const double INERTIA_START = 0.95;  // Higher initial inertia for better exploration
        private const double INERTIA_END = 0.2;     // Lower end inertia for better fine-tuning
        private const double COGNITIVE = 1.8;        // Reduced to prevent overshooting
        private const double SOCIAL = 2.2;          // Increased for better convergence
        private const int MAX_ITERATIONS = 50;      // Reduced for faster adaptation

        // Dynamic velocity limits with better defaults
        private double maxVelocityH = 15.0;        // Reduced for smoother movement
        private double maxVelocityV = 7.5;         // Half of horizontal for stability

        // Current state
        private int iteration;
        private double currentInertia;
        private double prevBestFitness;
        private int stagnationCount;
        private const int STAGNATION_THRESHOLD = 5;

        // Improved exploration vs exploitation balance
        private double explorationFactor = 1.0;
        private const double EXPLORATION_DECAY = 0.98;  // Slower decay

        // Signal strength thresholds adjusted for realism
        private const double EXCELLENT_SIGNAL = 70.0;  // More realistic threshold
        private const double GOOD_SIGNAL = 50.0;
        private const double FAIR_SIGNAL = 30.0;
        private const double POOR_SIGNAL = 10.0;

        // Tracking optimization
        private double searchCenterH = 0.0;
        private double searchCenterV = 0.0;
        private List<(double h, double v, double fitness)> historicalBests = new List<(double, double, double)>();
        private const int MAX_HISTORY = 10;

        // Convergence metrics
        public double ConvergenceRate { get; private set; } = 0;
        public double SearchRadius { get; private set; } = 180.0;

        public int ParticleCount => particles.Count;
        public double BestFitness => bestFitness;
        public double BestHorizontalAngle => bestHorizontalAngle;
        public double BestVerticalAngle => bestVerticalAngle;
        public int CurrentIteration => iteration;

        public ParticleSwarmOptimizer(int particleCount = 50)
        {
            particles = new List<Particle2D>();
            bestFitness = double.MinValue;
            prevBestFitness = double.MinValue;
            Reset();
            InitializeParticles(particleCount);
        }

        private void InitializeParticles(int count)
        {
            particles.Clear();

            // Artan yoğunlukta grid
            int gridSize = (int)Math.Sqrt(count * 0.8); // %80 grid bazlı
            double angleStepH = 360.0 / gridSize;
            double angleStepV = 90.0 / gridSize;

            for (int i = 0; i < gridSize; i++)
            {
                for (int j = 0; j < gridSize; j++)
                {
                    double h = (i * angleStepH + random.NextDouble() * angleStepH) % 360.0;
                    double v = Math.Min(90.0, j * angleStepV + random.NextDouble() * angleStepV);

                    particles.Add(new Particle2D
                    {
                        HorizontalPosition = h,
                        VerticalPosition = v,
                        HorizontalVelocity = 0,
                        VerticalVelocity = 0,
                        BestHorizontalPosition = h,
                        BestVerticalPosition = v,
                        BestFitness = double.MinValue
                    });
                }
            }

            // Kalan %20'yi rastgele dağıt
            while (particles.Count < count)
            {
                double h = random.NextDouble() * 360.0;
                double v = random.NextDouble() * 90.0;

                particles.Add(new Particle2D
                {
                    HorizontalPosition = h,
                    VerticalPosition = v,
                    HorizontalVelocity = 0,
                    VerticalVelocity = 0,
                    BestHorizontalPosition = h,
                    BestVerticalPosition = v,
                    BestFitness = double.MinValue
                });
            }
        }

        public (double h, double v) ScanStep(Func<double, double, double> fitnessFunc,
                                            double searchAreaH, double searchAreaV,
                                            double targetH, double targetV,
                                            bool useVelocity = true,
                                            double predictedH = -1, double predictedV = -1)
        {
            iteration++;

            // Adaptive inertia weight based on iteration and signal quality
            currentInertia = INERTIA_START - (INERTIA_START - INERTIA_END) *
                            (iteration / (double)MAX_ITERATIONS);

            // Dynamic search area adaptation
            explorationFactor *= EXPLORATION_DECAY;
            double effectiveSearchAreaH = searchAreaH * explorationFactor;
            double effectiveSearchAreaV = searchAreaV * explorationFactor;

            // Update particles with improved positioning
            foreach (var particle in particles)
            {
                if (iteration == 1 || random.NextDouble() < 0.05) // Reduced random repositioning
                {
                    RepositionParticle(particle, targetH, targetV, effectiveSearchAreaH,
                                     effectiveSearchAreaV, predictedH, predictedV);
                }

                // Fitness değerlendir
                double fitness = fitnessFunc(particle.HorizontalPosition, particle.VerticalPosition);

                if (fitness > particle.BestFitness)
                {
                    particle.BestFitness = fitness;
                    particle.BestHorizontalPosition = particle.HorizontalPosition;
                    particle.BestVerticalPosition = particle.VerticalPosition;
                }

                if (fitness > bestFitness)
                {
                    bestFitness = fitness;
                    bestHorizontalAngle = particle.HorizontalPosition;
                    bestVerticalAngle = particle.VerticalPosition;
                }
            }

            // Hız ve pozisyon güncellemelerini yap
            foreach (var particle in particles)
            {
                double r1 = random.NextDouble();
                double r2 = random.NextDouble();

                double cognitiveH = COGNITIVE * r1 * GetAngleDifference(particle.BestHorizontalPosition, particle.HorizontalPosition);
                double socialH = SOCIAL * r2 * GetAngleDifference(bestHorizontalAngle, particle.HorizontalPosition);

                double cognitiveV = COGNITIVE * r1 * (particle.BestVerticalPosition - particle.VerticalPosition);
                double socialV = SOCIAL * r2 * (bestVerticalAngle - particle.VerticalPosition);

                particle.HorizontalVelocity = currentInertia * particle.HorizontalVelocity + cognitiveH + socialH;
                particle.VerticalVelocity = currentInertia * particle.VerticalVelocity + cognitiveV + socialV;

                particle.HorizontalVelocity = Math.Max(-maxVelocityH, Math.Min(maxVelocityH, particle.HorizontalVelocity));
                particle.VerticalVelocity = Math.Max(-maxVelocityV, Math.Min(maxVelocityV, particle.VerticalVelocity));

                particle.HorizontalPosition = (particle.HorizontalPosition + particle.HorizontalVelocity + 360.0) % 360.0;
                particle.VerticalPosition = Math.Max(0, Math.Min(90.0, particle.VerticalPosition + particle.VerticalVelocity));
            }

            // Calculate convergence metrics
            CalculateConvergenceMetrics();

            return (bestHorizontalAngle, bestVerticalAngle);
        }

        private void CalculateConvergenceMetrics()
        {
            // Calculate fitness improvement rate
            double fitnessImprovement = bestFitness - prevBestFitness;
            prevBestFitness = bestFitness;

            // Update stagnation counter
            if (Math.Abs(fitnessImprovement) < 0.001)
            {
                stagnationCount++;
                if (stagnationCount > STAGNATION_THRESHOLD)
                {
                    // Reset exploration factor to escape local optima
                    explorationFactor = Math.Min(1.0, explorationFactor * 1.5);
                }
            }
            else
            {
                stagnationCount = 0;
            }

            // Calculate particle distribution
            CalculateSearchRadius();

            // Calculate overall convergence rate (0-1)
            double iterationProgress = Math.Min(1.0, iteration / (double)MAX_ITERATIONS);
            double fitnessProgress = Math.Min(1.0, bestFitness / 100.0);
            double distributionFactor = Math.Min(1.0, SearchRadius / 180.0);

            ConvergenceRate = (fitnessProgress * 0.5 +
                             (1.0 - distributionFactor) * 0.3 +
                             iterationProgress * 0.2);
        }

        private void RepositionParticle(Particle2D particle, double targetH, double targetV,
                                      double searchAreaH, double searchAreaV,
                                      double predictedH, double predictedV)
        {
            // Use predicted position if available
            double centerH = predictedH >= 0 ? predictedH : targetH;
            double centerV = predictedV >= 0 ? predictedV : targetV;

            // Adaptive standard deviation based on exploration factor
            double stdDevH = searchAreaH * 0.25 * explorationFactor;
            double stdDevV = searchAreaV * 0.25 * explorationFactor;

            // Calculate random offset using Gaussian distribution
            double offsetH = NextGaussian() * stdDevH;
            double offsetV = NextGaussian() * stdDevV;

            // Position particle with wrap-around for horizontal angle
            particle.HorizontalPosition = (centerH + offsetH + 360.0) % 360.0;
            particle.VerticalPosition = Math.Max(0, Math.Min(90, centerV + offsetV));

            // Reset velocities with smaller initial values
            particle.HorizontalVelocity = (random.NextDouble() - 0.5) * maxVelocityH * 0.2;
            particle.VerticalVelocity = (random.NextDouble() - 0.5) * maxVelocityV * 0.2;
        }

        private void CalculateSearchRadius()
        {
            // Calculate the effective search radius based on particle distribution
            if (particles.Count < 2) return;

            // Calculate how spread out the particles are, especially the best-performing ones
            var topParticles = particles.OrderByDescending(p => p.BestFitness)
                                       .Take(particles.Count / 3).ToList();

            double totalHDist = 0;
            double totalVDist = 0;
            int count = 0;

            foreach (var p1 in topParticles)
            {
                foreach (var p2 in topParticles)
                {
                    if (p1 != p2)
                    {
                        double hDiff = Math.Abs(GetAngleDifference(p1.HorizontalPosition, p2.HorizontalPosition));
                        double vDiff = Math.Abs(p1.VerticalPosition - p2.VerticalPosition);

                        totalHDist += hDiff;
                        totalVDist += vDiff;
                        count++;
                    }
                }
            }

            // Average distance between particles
            if (count > 0)
            {
                double avgHDist = totalHDist / count;
                double avgVDist = totalVDist / count;

                // Set search radius as the average of horizontal and vertical spread
                SearchRadius = Math.Max(5.0, (avgHDist + avgVDist) / 2.0);
            }
        }

        private double GetAngleDifference(double targetAngle, double currentAngle)
        {
            // Calculate minimum angle difference considering 0-360 wrap-around
            double diff = ((targetAngle - currentAngle + 540.0) % 360.0) - 180.0;
            return diff;
        }

        private void InjectDiversity(double searchAreaH, double searchAreaV, double targetH, double targetV)
        {
            // Find the worst performing 30% of particles for repositioning
            int refreshCount = particles.Count / 3;
            var worstParticles = particles
                .OrderBy(p => p.BestFitness)
                .Take(refreshCount)
                .ToList();

            // Reposition these particles with a multi-strategy approach
            foreach (var particle in worstParticles)
            {
                // Strategy selection based on position in the refresh list
                int strategyIndex = worstParticles.IndexOf(particle) % 3;

                switch (strategyIndex)
                {
                    case 0: // Close to target - fine search
                        double randomOffsetH = (random.NextDouble() - 0.5) * searchAreaH * 0.5;
                        double randomOffsetV = (random.NextDouble() - 0.5) * searchAreaV * 0.5;

                        particle.HorizontalPosition = (targetH + randomOffsetH + 360.0) % 360.0;
                        particle.VerticalPosition = Math.Max(0, Math.Min(90.0, targetV + randomOffsetV));
                        break;

                    case 1: // Around best known position - exploitation
                        double bestOffsetH = (random.NextDouble() - 0.5) * searchAreaH * 0.3;
                        double bestOffsetV = (random.NextDouble() - 0.5) * searchAreaV * 0.3;

                        particle.HorizontalPosition = (bestHorizontalAngle + bestOffsetH + 360.0) % 360.0;
                        particle.VerticalPosition = Math.Max(0, Math.Min(90.0, bestVerticalAngle + bestOffsetV));
                        break;

                    case 2: // Wide area search - exploration
                        double wideOffsetH = (random.NextDouble() - 0.5) * searchAreaH;
                        double wideOffsetV = (random.NextDouble() - 0.5) * searchAreaV;

                        particle.HorizontalPosition = (targetH + wideOffsetH + 360.0) % 360.0;
                        particle.VerticalPosition = Math.Max(0, Math.Min(90.0, targetV + wideOffsetV));
                        break;
                }

                // Reset velocity with random direction but controlled magnitude
                particle.HorizontalVelocity = (random.NextDouble() - 0.5) * maxVelocityH;
                particle.VerticalVelocity = (random.NextDouble() - 0.5) * maxVelocityV;

                // Reset personal best for this particle to force exploration
                particle.BestFitness = double.MinValue;
            }
        }

        private void AdaptToSignalQuality(double signalQuality, double searchAreaH, double searchAreaV)
        {
            // Dynamic adaptation based on signal quality
            if (signalQuality >= EXCELLENT_SIGNAL)
            {
                // Fine-tune with smaller steps for excellent signals
                maxVelocityH = searchAreaH * 0.05;
                maxVelocityV = searchAreaV * 0.05;
                currentInertia = Math.Max(INERTIA_END, currentInertia * 0.9); // Reduce inertia for fine-tuning
            }
            else if (signalQuality >= GOOD_SIGNAL)
            {
                // Moderate movement for good signals
                maxVelocityH = searchAreaH * 0.1;
                maxVelocityV = searchAreaV * 0.1;
            }
            else if (signalQuality >= FAIR_SIGNAL)
            {
                // More aggressive search for fair signals
                maxVelocityH = searchAreaH * 0.15;
                maxVelocityV = searchAreaV * 0.15;
                currentInertia = Math.Min(INERTIA_START, currentInertia * 1.1); // Increase inertia for more exploration
            }
            else
            {
                // Wide search for poor signals with high inertia
                maxVelocityH = searchAreaH * 0.2;
                maxVelocityV = searchAreaV * 0.2;
                currentInertia = INERTIA_START; // High inertia for extensive exploration
            }
        }

        public void Reset()
        {
            iteration = 0;
            currentInertia = INERTIA_START;
            bestFitness = double.MinValue;
            prevBestFitness = double.MinValue;
            stagnationCount = 0;
            explorationFactor = 1.0;
            historicalBests.Clear();
            ConvergenceRate = 0;
            SearchRadius = 180.0;
        }

        public void PartialReset()
        {
            // Reset half of the particles randomly
            for (int i = 0; i < particles.Count / 2; i++)
            {
                int index = random.Next(particles.Count);
                var particle = particles[index];

                // Reset positions with random values
                particle.HorizontalPosition = random.NextDouble() * 360.0;
                particle.VerticalPosition = random.NextDouble() * 90.0;

                // Reset velocities with controlled random values
                particle.HorizontalVelocity = (random.NextDouble() * 2 - 1) * maxVelocityH * 0.5;
                particle.VerticalVelocity = (random.NextDouble() * 2 - 1) * maxVelocityV * 0.5;

                // Reset personal best
                particle.BestHorizontalPosition = particle.HorizontalPosition;
                particle.BestVerticalPosition = particle.VerticalPosition;
                particle.BestFitness = double.MinValue;
            }

            // Reset optimization parameters
            currentInertia = INERTIA_START;
            ConvergenceRate = 0;
            iteration = 0;
            stagnationCount = 0;
            explorationFactor = 1.0;
            SearchRadius = 180.0;
        }

        /// <summary>
        /// Returns current state of all particles for visualization
        /// </summary>
        public IEnumerable<(double HorizontalPosition, double VerticalPosition,
                           double HorizontalVelocity, double VerticalVelocity)> GetParticles()
        {
            return particles.Select(p => (
                p.HorizontalPosition,
                p.VerticalPosition,
                p.HorizontalVelocity,
                p.VerticalVelocity
            ));
        }

        /// <summary>
        /// Generates a random number from a standard normal distribution using Box-Muller transform
        /// </summary>
        private double NextGaussian()
        {
            double u1, u2;
            do
            {
                u1 = 2.0 * random.NextDouble() - 1.0;
                u2 = 2.0 * random.NextDouble() - 1.0;
            }
            while (u1 * u1 + u2 * u2 >= 1.0 || u1 * u1 + u2 * u2 == 0.0);

            double multiplier = Math.Sqrt(-2.0 * Math.Log(u1 * u1 + u2 * u2) / (u1 * u1 + u2 * u2));
            return u1 * multiplier;
        }

        /// <summary>
        /// Represents a particle in 2D space with position, velocity, and memory of best position
        /// </summary>
        private class Particle2D
        {
            public double HorizontalPosition { get; set; }
            public double VerticalPosition { get; set; }
            public double HorizontalVelocity { get; set; }
            public double VerticalVelocity { get; set; }
            public double BestHorizontalPosition { get; set; }
            public double BestVerticalPosition { get; set; }
            public double BestFitness { get; set; }
        }
    }
}