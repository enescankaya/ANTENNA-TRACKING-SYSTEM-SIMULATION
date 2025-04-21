using System;
using System.Collections.Generic;
using System.Linq;

namespace Project.Services
{
    public class ParticleSwarmOptimizer
    {
        private readonly Random random = new Random();
        private readonly List<Particle2D> particles;
        private double bestHorizontalAngle;
        private double bestVerticalAngle;
        private double bestFitness;

        // PSO parameters tuned for antenna tracking
        private const double INERTIA_START = 0.9;
        private const double INERTIA_END = 0.4;
        private const double COGNITIVE = 2.0;  // Weight for particle's own best
        private const double SOCIAL = 2.5;     // Weight for global best (increased for faster convergence)
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

        public int ParticleCount => particles.Count;
        public double BestFitness => bestFitness;

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

            // Grid-based initialization for wide coverage
            int gridPoints = (int)Math.Sqrt(count * 0.7); // Use 70% of particles for grid
            double gridStepH = 360.0 / gridPoints;
            double gridStepV = 90.0 / gridPoints;

            int particleIndex = 0;

            // Create grid-based particles with better distribution
            for (int i = 0; i < gridPoints && particleIndex < count; i++)
            {
                for (int j = 0; j < gridPoints && particleIndex < count; j++)
                {
                    // Distribute particles across full 360-degree range
                    double h = (i * gridStepH + random.NextDouble() * gridStepH) % 360.0;
                    double v = Math.Min(90.0, j * gridStepV + random.NextDouble() * gridStepV);

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

            // Fill remaining with uniform random distribution
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

        public (double h, double v) ScanStep(Func<double, double, double> fitnessFunc, double searchAreaH, double searchAreaV, double targetH, double targetV)
        {
            iteration++;

            // Update inertia weight
            currentInertia = INERTIA_START - (INERTIA_START - INERTIA_END) * (iteration / (double)MAX_ITERATIONS);

            // Adjust velocity limits based on search area
            maxVelocityH = searchAreaH * 0.15;
            maxVelocityV = searchAreaV * 0.15;

            // Reset stagnation if improvement is observed
            if (bestFitness > prevBestFitness + 1.0)
            {
                stagnationCount = 0;
                prevBestFitness = bestFitness;
            }
            else
            {
                stagnationCount++;
            }

            // Inject diversity if stagnation detected
            if (stagnationCount > STAGNATION_THRESHOLD)
            {
                InjectDiversity(searchAreaH, searchAreaV, targetH, targetV);
                stagnationCount = 0;
            }

            // Update all particles
            foreach (var particle in particles)
            {
                // Evaluate current position
                double fitness = fitnessFunc(particle.HorizontalPosition, particle.VerticalPosition);

                // Update particle best
                if (fitness > particle.BestFitness)
                {
                    particle.BestFitness = fitness;
                    particle.BestHorizontalPosition = particle.HorizontalPosition;
                    particle.BestVerticalPosition = particle.VerticalPosition;
                }

                // Update global best
                if (fitness > bestFitness)
                {
                    bestFitness = fitness;
                    bestHorizontalAngle = particle.HorizontalPosition;
                    bestVerticalAngle = particle.VerticalPosition;
                    searchCenterH = bestHorizontalAngle;
                    searchCenterV = bestVerticalAngle;

                    historicalBests.Add((bestHorizontalAngle, bestVerticalAngle, bestFitness));
                    if (historicalBests.Count > MAX_HISTORY)
                        historicalBests.RemoveAt(0);
                }

                // Calculate cognitive and social influence
                double r1 = random.NextDouble();
                double r2 = random.NextDouble();

                double cognitiveH = COGNITIVE * r1 * GetAngleDifference(particle.BestHorizontalPosition, particle.HorizontalPosition);
                double socialH = SOCIAL * r2 * GetAngleDifference(bestHorizontalAngle, particle.HorizontalPosition);

                double cognitiveV = COGNITIVE * r1 * (particle.BestVerticalPosition - particle.VerticalPosition);
                double socialV = SOCIAL * r2 * (bestVerticalAngle - particle.VerticalPosition);

                // Update velocities
                particle.HorizontalVelocity = currentInertia * particle.HorizontalVelocity + cognitiveH + socialH;
                particle.VerticalVelocity = currentInertia * particle.VerticalVelocity + cognitiveV + socialV;

                // Enforce velocity limits
                particle.HorizontalVelocity = Math.Max(-maxVelocityH, Math.Min(maxVelocityH, particle.HorizontalVelocity));
                particle.VerticalVelocity = Math.Max(-maxVelocityV, Math.Min(maxVelocityV, particle.VerticalVelocity));

                // Update positions
                particle.HorizontalPosition = (particle.HorizontalPosition + particle.HorizontalVelocity + 360.0) % 360.0;
                particle.VerticalPosition = Math.Max(0, Math.Min(90.0, particle.VerticalPosition + particle.VerticalVelocity));
            }

            // Apply adaptive behavior
            AdaptToSignalQuality(bestFitness, searchAreaH, searchAreaV);
            explorationFactor *= EXPLORATION_DECAY;

            return (bestHorizontalAngle, bestVerticalAngle);
        }

        private double GetAngleDifference(double targetAngle, double currentAngle)
        {
            double diff = ((targetAngle - currentAngle + 540.0) % 360.0) - 180.0;
            return diff;
        }

        private void InjectDiversity(double searchAreaH, double searchAreaV, double targetH, double targetV)
        {
            // Find the worst performing 30% of particles
            int refreshCount = particles.Count / 3;
            var worstParticles = particles
                .OrderBy(p => p.BestFitness)
                .Take(refreshCount)
                .ToList();

            // Reposition these particles around the target position with some randomness
            foreach (var particle in worstParticles)
            {
                double randomOffsetH = (random.NextDouble() - 0.5) * searchAreaH;
                double randomOffsetV = (random.NextDouble() - 0.5) * searchAreaV;

                particle.HorizontalPosition = (targetH + randomOffsetH + 360.0) % 360.0;
                particle.VerticalPosition = Math.Max(0, Math.Min(90.0, targetV + randomOffsetV));

                particle.HorizontalVelocity = (random.NextDouble() - 0.5) * maxVelocityH;
                particle.VerticalVelocity = (random.NextDouble() - 0.5) * maxVelocityV;

                particle.BestFitness = double.MinValue;
            }
        }

        private void AdaptToSignalQuality(double signalQuality, double searchAreaH, double searchAreaV)
        {
            // Adaptive behavior based on signal quality
            if (signalQuality >= EXCELLENT_SIGNAL)
            {
                // Fine-tune with smaller steps
                maxVelocityH = searchAreaH * 0.05;
                maxVelocityV = searchAreaV * 0.05;
            }
            else if (signalQuality >= GOOD_SIGNAL)
            {
                // Moderate movement
                maxVelocityH = searchAreaH * 0.1;
                maxVelocityV = searchAreaV * 0.1;
            }
            else if (signalQuality >= FAIR_SIGNAL)
            {
                // More aggressive search
                maxVelocityH = searchAreaH * 0.15;
                maxVelocityV = searchAreaV * 0.15;
            }
            else
            {
                // Wide search for poor signals
                maxVelocityH = searchAreaH * 0.2;
                maxVelocityV = searchAreaV * 0.2;
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
            particles.Clear();
        }

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
