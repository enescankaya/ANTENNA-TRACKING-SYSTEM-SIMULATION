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
        private const double W_START = 0.9; // Başlangıç atalet ağırlığı
        private const double W_END = 0.4;   // Bitiş atalet ağırlığı
        private const double C1 = 2.0;      // Bilişsel bileşen
        private const double C2 = 2.0;      // Sosyal bileşen
        private double currentW;
        private int iteration;
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
            bestFitness = double.MinValue;
            currentInertia = INERTIA_START;
            currentW = W_START;
            InitializeParticles(particleCount);
        }

        private void InitializeParticles(int count)
        {
            particles.Clear();
            for (int i = 0; i < count; i++)
            {
                // Başlangıçta tüm alanı kapsayacak şekilde dağıt
                double h = random.NextDouble() * 360.0;
                double v = random.NextDouble() * 90.0;
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

        // PSO ile bir sonraki açıları döndür, particles listesini günceller
        public (double h, double v) ScanStep(Func<double, double, double> fitnessFunc, double searchAreaH, double searchAreaV)
        {
            iteration++;
            currentW = W_START - (W_START - W_END) * (iteration / (double)MAX_ITERATIONS);

            // Grid-based systematic search pattern
            if (iteration == 1)
            {
                double gridH = 360.0 / Math.Sqrt(particles.Count);
                double gridV = 90.0 / Math.Sqrt(particles.Count);

                int index = 0;
                for (double h = 0; h < 360.0; h += gridH)
                {
                    for (double v = 0; v < 90.0; v += gridV)
                    {
                        if (index < particles.Count)
                        {
                            particles[index].HorizontalPosition = h;
                            particles[index].VerticalPosition = v;
                            particles[index].HorizontalVelocity = 0;
                            particles[index].VerticalVelocity = 0;
                            index++;
                        }
                    }
                }
            }

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
                }

                // Update velocities with momentum
                double r1 = random.NextDouble();
                double r2 = random.NextDouble();

                particle.HorizontalVelocity = currentW * particle.HorizontalVelocity +
                    C1 * r1 * (particle.BestHorizontalPosition - particle.HorizontalPosition) +
                    C2 * r2 * (bestHorizontalAngle - particle.HorizontalPosition);

                particle.VerticalVelocity = currentW * particle.VerticalVelocity +
                    C1 * r1 * (particle.BestVerticalPosition - particle.VerticalPosition) +
                    C2 * r2 * (bestVerticalAngle - particle.VerticalPosition);

                // Velocity bounds
                double maxVelH = searchAreaH * 0.1;
                double maxVelV = searchAreaV * 0.1;

                particle.HorizontalVelocity = Math.Max(-maxVelH, Math.Min(maxVelH, particle.HorizontalVelocity));
                particle.VerticalVelocity = Math.Max(-maxVelV, Math.Min(maxVelV, particle.VerticalVelocity));

                // Update positions with bounds checking
                particle.HorizontalPosition = (particle.HorizontalPosition + particle.HorizontalVelocity + 360.0) % 360.0;
                particle.VerticalPosition = Math.Max(0, Math.Min(90.0, particle.VerticalPosition + particle.VerticalVelocity));
            }

            return (bestHorizontalAngle, bestVerticalAngle);
        }

        // Tarama alanını sıfırla ve parçacıkları yeniden başlat
        public void Reset()
        {
            iteration = 0;
            currentW = W_START;
            bestFitness = double.MinValue;
            particles.Clear();
            InitializeParticles(40);
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
