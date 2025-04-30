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
        private const double INERTIA_START = 0.9;
        private const double INERTIA_END = 0.3;  // Lower end inertia for better fine-tuning
        private const double COGNITIVE = 2.0;    // Weight for particle's own best
        private const double SOCIAL = 2.5;       // Weight for global best (higher for faster convergence)
        private const int MAX_ITERATIONS = 100;

        // Dynamic velocity limits
        private double maxVelocityH = 20.0;
        private double maxVelocityV = 10.0;

        // Current state
        private int iteration;
        private double currentInertia;
        private double prevBestFitness;
        private int stagnationCount;
        private const int STAGNATION_THRESHOLD = 5;

        // Exploration vs exploitation balance
        private double explorationFactor = 1.0;
        private const double EXPLORATION_DECAY = 0.95;

        // Signal strength thresholds for adaptive behavior
        private const double EXCELLENT_SIGNAL = 80.0;
        private const double GOOD_SIGNAL = 60.0;
        private const double FAIR_SIGNAL = 40.0;
        private const double POOR_SIGNAL = 20.0;

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

            // Enhanced initialization strategy:
            // - Grid-based particles for thorough coverage (70%)
            // - Randomized particles for exploration (30%)
            int gridPoints = (int)Math.Sqrt(count * 0.7);
            double gridStepH = 360.0 / gridPoints;
            double gridStepV = 90.0 / gridPoints;

            int particleIndex = 0;

            // Create grid-based particles with deliberate distribution
            for (int i = 0; i < gridPoints && particleIndex < count; i++)
            {
                for (int j = 0; j < gridPoints && particleIndex < count; j++)
                {
                    // Distribute particles across full 360-degree horizontal range
                    // and 90-degree vertical range with small randomness
                    double h = (i * gridStepH + random.NextDouble() * (gridStepH * 0.5)) % 360.0;
                    double v = Math.Min(90.0, j * gridStepV + random.NextDouble() * (gridStepV * 0.5));

                    particles.Add(new Particle2D
                    {
                        HorizontalPosition = h,
                        VerticalPosition = v,
                        HorizontalVelocity = (random.NextDouble() - 0.5) * maxVelocityH * 0.5,
                        VerticalVelocity = (random.NextDouble() - 0.5) * maxVelocityV * 0.5,
                        BestHorizontalPosition = h,
                        BestVerticalPosition = v,
                        BestFitness = double.MinValue
                    });

                    particleIndex++;
                }
            }

            // Fill remaining with uniform random distribution for exploration
            while (particleIndex < count)
            {
                double h = random.NextDouble() * 360.0;
                double v = random.NextDouble() * 90.0;

                particles.Add(new Particle2D
                {
                    HorizontalPosition = h,
                    VerticalPosition = v,
                    HorizontalVelocity = (random.NextDouble() - 0.5) * maxVelocityH,
                    VerticalVelocity = (random.NextDouble() - 0.5) * maxVelocityV,
                    BestHorizontalPosition = h,
                    BestVerticalPosition = v,
                    BestFitness = double.MinValue
                });

                particleIndex++;
            }
        }

        public (double h, double v) ScanStep(Func<double, double, double> fitnessFunc,
                                            double searchAreaH, double searchAreaV,
                                            double targetH, double targetV,
                                            bool useVelocity = true,
                                            double predictedH = -1, double predictedV = -1)
        {
            iteration++;

            // Hedef etrafında parçacıkları konumlandır
            foreach (var particle in particles)
            {
                if (iteration == 1 || random.NextDouble() < 0.1) // %10 şans ile rastgele konum
                {
                    double offsetH = (random.NextDouble() - 0.5) * searchAreaH;
                    double offsetV = (random.NextDouble() - 0.5) * searchAreaV;

                    particle.HorizontalPosition = (targetH + offsetH + 360.0) % 360.0;
                    particle.VerticalPosition = Math.Max(0, Math.Min(90, targetV + offsetV));
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

            return (bestHorizontalAngle, bestVerticalAngle);
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