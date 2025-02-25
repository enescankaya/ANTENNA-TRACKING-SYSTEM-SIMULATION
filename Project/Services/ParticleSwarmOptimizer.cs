using System;
using System.Collections.Generic;

namespace Project.Services
{
    public class ParticleSwarmOptimizer
    {
        private readonly Random random = new Random();
        private readonly List<Particle> particles;
        private readonly int particleCount;
        private double globalBestPosition;
        private double globalBestFitness;

        public ParticleSwarmOptimizer(int particleCount = 20)
        {
            this.particleCount = particleCount;
            this.particles = new List<Particle>();
            InitializeParticles();
        }

        private void InitializeParticles()
        {
            particles.Clear();
            for (int i = 0; i < particleCount; i++)
            {
                particles.Add(new Particle
                {
                    Position = random.NextDouble() * 360,
                    Velocity = (random.NextDouble() - 0.5) * 10,
                    BestPosition = 0,
                    BestFitness = double.MinValue
                });
            }
        }

        public double GetNextAngle(double currentSignalStrength)
        {
            UpdateParticles(currentSignalStrength);
            return globalBestPosition;
        }

        private void UpdateParticles(double fitness)
        {
            const double w = 0.729; // Inertia
            const double c1 = 1.49; // Cognitive parameter
            const double c2 = 1.49; // Social parameter

            foreach (var particle in particles)
            {
                // Update personal best
                if (fitness > particle.BestFitness)
                {
                    particle.BestFitness = fitness;
                    particle.BestPosition = particle.Position;

                    // Update global best
                    if (fitness > globalBestFitness)
                    {
                        globalBestFitness = fitness;
                        globalBestPosition = particle.Position;
                    }
                }

                // Update velocity and position
                double r1 = random.NextDouble();
                double r2 = random.NextDouble();

                particle.Velocity = w * particle.Velocity +
                                  c1 * r1 * (particle.BestPosition - particle.Position) +
                                  c2 * r2 * (globalBestPosition - particle.Position);

                particle.Position += particle.Velocity;

                // Keep within 0-360 range
                particle.Position = (particle.Position + 360) % 360;
            }
        }

        private class Particle
        {
            public double Position { get; set; }
            public double Velocity { get; set; }
            public double BestPosition { get; set; }
            public double BestFitness { get; set; }
        }
    }
}
