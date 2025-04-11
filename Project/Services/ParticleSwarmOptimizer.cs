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

        // Adjusted PSO parameters
        private const double INERTIA_START = 0.9;
        private const double INERTIA_END = 0.2;   // Lower end inertia
        private double currentInertia;
        private const double COGNITIVE = 2.0;
        private const double SOCIAL = 2.0;
        private double MAX_VELOCITY_H = 20.0;  // Increased for faster response
        private double MAX_VELOCITY_V = 10.0;  // Increased for faster response
        private int iteration = 0;
        private const int MAX_ITERATIONS = 100;

        // Signal strength thresholds
        private const double EXCELLENT_SIGNAL = 80.0;
        private const double GOOD_SIGNAL = 60.0;
        private const double POOR_SIGNAL = 40.0;
        private const double MIN_SIGNAL = 20.0;

        // Search area management
        private const double MIN_SEARCH_AREA = 30.0;
        private const double SEARCH_AREA_REDUCTION = 0.8;
        private double currentSearchArea = 360.0;
        private double searchCenterH = 180.0;
        private double searchCenterV = 45.0;
        private const double CONVERGENCE_THRESHOLD = 5.0;

        public ParticleSwarmOptimizer(int particleCount = 30)
        {
            particles = new List<Particle2D>();
            InitializeParticles(particleCount);
            bestFitness = double.MinValue;
            currentInertia = INERTIA_START;
        }

        private void InitializeParticles(int count)
        {
            particles.Clear();
            for (int i = 0; i < count; i++)
            {
                // Initialize within current search area
                double halfAreaH = currentSearchArea / 2;
                double halfAreaV = Math.Min(45.0, currentSearchArea / 4);

                double h = ((searchCenterH + halfAreaH + (random.NextDouble() * currentSearchArea)) % 360.0);
                double v = Math.Max(0, Math.Min(90, searchCenterV + (random.NextDouble() - 0.5) * halfAreaV * 2));

                particles.Add(new Particle2D
                {
                    HorizontalPosition = h,
                    VerticalPosition = v,
                    HorizontalVelocity = (random.NextDouble() - 0.5) * 2 * MAX_VELOCITY_H,
                    VerticalVelocity = (random.NextDouble() - 0.5) * 2 * MAX_VELOCITY_V,
                    BestHorizontalPosition = h,
                    BestVerticalPosition = v,
                    BestFitness = double.MinValue
                });
            }
        }

        public (double horizontalAngle, double verticalAngle) GetNextPosition(
            double currentFitness,
            double searchRadius,
            double targetH,
            double targetV)
        {
            UpdateGlobalBest(currentFitness);

            // Dynamic search area adjustment
            double spreadH = Math.Min(searchRadius, 180.0);
            double spreadV = Math.Min(searchRadius / 2, 45.0);

            // Adjust inertia based on signal quality
            currentInertia = currentFitness > 70 ?
                INERTIA_END :
                INERTIA_START - (currentFitness / 100.0) * (INERTIA_START - INERTIA_END);

            foreach (var particle in particles)
            {
                // Update personal best
                if (currentFitness > particle.BestFitness)
                {
                    particle.BestFitness = currentFitness;
                    particle.BestHorizontalPosition = particle.HorizontalPosition;
                    particle.BestVerticalPosition = particle.VerticalPosition;
                }

                UpdateParticleVelocityWithTarget(particle, targetH, targetV, spreadH, spreadV);
                UpdateParticlePosition(particle, searchRadius);
            }

            // Concentrate particles when signal is strong
            if (currentFitness > 70)
            {
                ConcentrateParticles(targetH, targetV, spreadH * 0.3, spreadV * 0.3);
            }

            return (bestHorizontalAngle, bestVerticalAngle);
        }

        private void UpdateParticleVelocityWithTarget(
            Particle2D particle,
            double targetH,
            double targetV,
            double spreadH,
            double spreadV)
        {
            double r1 = random.NextDouble();
            double r2 = random.NextDouble();
            double r3 = random.NextDouble();

            // Hedef konuma doğru bileşen ekle
            particle.HorizontalVelocity = currentInertia * particle.HorizontalVelocity +
                COGNITIVE * r1 * (particle.BestHorizontalPosition - particle.HorizontalPosition) +
                SOCIAL * r2 * (bestHorizontalAngle - particle.HorizontalPosition) +
                SOCIAL * r3 * (targetH - particle.HorizontalPosition);

            particle.VerticalVelocity = currentInertia * particle.VerticalVelocity +
                COGNITIVE * r1 * (particle.BestVerticalPosition - particle.VerticalPosition) +
                SOCIAL * r2 * (bestVerticalAngle - particle.VerticalPosition) +
                SOCIAL * r3 * (targetV - particle.VerticalPosition);

            // Hız sınırlaması
            particle.HorizontalVelocity = Math.Max(-spreadH, Math.Min(spreadH, particle.HorizontalVelocity));
            particle.VerticalVelocity = Math.Max(-spreadV, Math.Min(spreadV, particle.VerticalVelocity));
        }

        public void IncreaseSearchArea()
        {
            // Parçacıkları daha geniş alana yay
            foreach (var particle in particles)
            {
                if (random.NextDouble() < 0.3) // %30 olasılıkla
                {
                    double randomAngle = random.NextDouble() * 360;
                    double randomElevation = random.NextDouble() * 90;

                    particle.HorizontalPosition = randomAngle;
                    particle.VerticalPosition = randomElevation;

                    // Hızları da rastgele ayarla
                    particle.HorizontalVelocity = (random.NextDouble() - 0.5) * MAX_VELOCITY_H * 2;
                    particle.VerticalVelocity = (random.NextDouble() - 0.5) * MAX_VELOCITY_V * 2;
                }
            }
        }

        private void AdjustParameters(double fitness)
        {
            if (fitness > 80)
            {
                // Fine-tuning mode
                currentInertia *= 0.8;
                MAX_VELOCITY_H = 5.0;
                MAX_VELOCITY_V = 3.0;
            }
            else if (fitness < 40)
            {
                // Exploration mode
                currentInertia = INERTIA_START;
                MAX_VELOCITY_H = 15.0;
                MAX_VELOCITY_V = 8.0;
            }
        }

        private void ConcentrateParticles(double targetH, double targetV, double spreadH, double spreadV)
        {
            foreach (var particle in particles)
            {
                if (random.NextDouble() < 0.3) // 30% chance to reposition
                {
                    double angle = random.NextDouble() * Math.PI * 2;
                    double radiusH = random.NextDouble() * spreadH;
                    double radiusV = random.NextDouble() * spreadV;

                    particle.HorizontalPosition = (targetH + radiusH * Math.Cos(angle) + 360.0) % 360.0;
                    particle.VerticalPosition = Math.Max(0, Math.Min(90,
                        targetV + radiusV * Math.Sin(angle)));
                }
            }
        }

        private void UpdateParticlePosition(Particle2D particle, double scanArea)
        {
            // Update horizontal position with boundary wrapping
            particle.HorizontalPosition = (particle.HorizontalPosition + particle.HorizontalVelocity + 360.0) % 360.0;

            // Update vertical position with boundary clamping
            particle.VerticalPosition = Math.Max(0, Math.Min(90,
                particle.VerticalPosition + particle.VerticalVelocity));

            // Constrain particles within current scan area around best position
            if (scanArea < 360.0)
            {
                double halfScanArea = scanArea / 2.0;
                double diffH = Math.Abs(particle.HorizontalPosition - bestHorizontalAngle);
                if (diffH > halfScanArea)
                {
                    particle.HorizontalPosition = bestHorizontalAngle +
                        (random.NextDouble() * 2 - 1) * halfScanArea;
                }
            }
        }

        private void UpdateGlobalBest(double fitness)
        {
            if (fitness > bestFitness)
            {
                bestFitness = fitness;
                var bestParticle = particles.OrderByDescending(p => p.BestFitness).First();
                bestHorizontalAngle = bestParticle.HorizontalPosition;
                bestVerticalAngle = bestParticle.VerticalPosition;
            }
        }

        public void UpdateSearchArea(double signalStrength, double currentH, double currentV)
        {
            if (signalStrength > bestFitness)
            {
                searchCenterH = currentH;
                searchCenterV = currentV;

                // Gradually reduce search area as signal improves
                if (signalStrength > 80)
                    currentSearchArea = Math.Max(MIN_SEARCH_AREA, currentSearchArea * 0.8);
                else if (signalStrength > 60)
                    currentSearchArea = Math.Max(MIN_SEARCH_AREA * 2, currentSearchArea * 0.9);
            }
            else if (signalStrength < bestFitness * 0.7) // Signal degraded significantly
            {
                currentSearchArea = Math.Min(360.0, currentSearchArea * 1.5);
            }
        }

        public void Reset()
        {
            bestFitness = double.MinValue;
            currentSearchArea = 360.0;
            searchCenterH = 180.0;
            searchCenterV = 45.0;
            iteration = 0;
            currentInertia = INERTIA_START;
            InitializeParticles(particles.Count);
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
