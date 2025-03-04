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
        private double searchRadius; // Arama yarıçapı
        private double inertiaWeight; // Dinamik inertia weight
        private const double MIN_SEARCH_RADIUS = 10.0; // Minimum arama yarıçapı (derece)
        private const double MAX_VELOCITY = 20.0; // Maksimum parçacık hızı
        private const double CONVERGENCE_THRESHOLD = 5.0; // Yakınsama eşiği
        private int stagnationCounter; // Yakınsama sayacı

        public ParticleSwarmOptimizer(int particleCount = 30)
        {
            this.particleCount = particleCount;
            this.particles = new List<Particle>();
            this.searchRadius = 360.0; // Başlangıçta tam daire
            this.inertiaWeight = 0.9; // Başlangıç inertia değeri
            InitializeParticles();
        }

        private void InitializeParticles()
        {
            particles.Clear();
            globalBestFitness = double.MinValue;
            stagnationCounter = 0;

            // Mevcut en iyi pozisyon etrafında parçacıkları dağıt
            double baseAngle = (globalBestFitness > double.MinValue) ? globalBestPosition : 0;

            for (int i = 0; i < particleCount; i++)
            {
                // Parçacıkları arama yarıçapı içinde rastgele dağıt
                double angle = baseAngle + (random.NextDouble() - 0.5) * 2 * searchRadius;
                angle = (angle + 360) % 360; // 0-360 aralığında tut

                particles.Add(new Particle
                {
                    Position = angle,
                    Velocity = (random.NextDouble() - 0.5) * MAX_VELOCITY,
                    BestPosition = angle,
                    BestFitness = double.MinValue
                });
            }
        }

        public double GetNextAngle(double currentSignalStrength)
        {
            UpdateParticleSwarm(currentSignalStrength);
            return globalBestPosition;
        }

        private void UpdateParticleSwarm(double fitness)
        {
            bool improved = false;
            double previousBest = globalBestFitness;

            // Cognitive ve social parametreler
            const double c1 = 1.5;
            const double c2 = 1.5;

            foreach (var particle in particles)
            {
                // Kişisel en iyi güncelleme
                if (fitness > particle.BestFitness)
                {
                    particle.BestFitness = fitness;
                    particle.BestPosition = particle.Position;

                    // Global en iyi güncelleme
                    if (fitness > globalBestFitness)
                    {
                        globalBestFitness = fitness;
                        globalBestPosition = particle.Position;
                        improved = true;
                    }
                }

                // Hız ve pozisyon güncelleme
                double r1 = random.NextDouble();
                double r2 = random.NextDouble();

                // Yeni hız hesaplama
                double newVelocity = inertiaWeight * particle.Velocity +
                                   c1 * r1 * (particle.BestPosition - particle.Position) +
                                   c2 * r2 * (globalBestPosition - particle.Position);

                // Hız sınırlama
                newVelocity = Math.Max(-MAX_VELOCITY, Math.Min(MAX_VELOCITY, newVelocity));
                particle.Velocity = newVelocity;

                // Pozisyon güncelleme
                particle.Position = (particle.Position + particle.Velocity + 360) % 360;
            }

            // Arama stratejisi güncelleme
            UpdateSearchStrategy(improved, previousBest);
        }

        private void UpdateSearchStrategy(bool improved, double previousBest)
        {
            if (improved)
            {
                stagnationCounter = 0;
                // İyileşme varsa arama yarıçapını kademeli olarak azalt
                searchRadius = Math.Max(MIN_SEARCH_RADIUS, searchRadius * 0.95);
                // Daha hassas arama için inertia'yı azalt
                inertiaWeight = Math.Max(0.4, inertiaWeight * 0.995);
            }
            else
            {
                stagnationCounter++;
                // Belirli bir süre iyileşme yoksa arama alanını genişlet
                if (stagnationCounter > 10)
                {
                    searchRadius = Math.Min(360.0, searchRadius * 1.1);
                    inertiaWeight = Math.Min(0.9, inertiaWeight * 1.05);

                    // Yakınsama kontrolü
                    if (CheckConvergence())
                    {
                        ReinitializeParticles();
                    }
                }
            }
        }

        private bool CheckConvergence()
        {
            // Parçacıkların ne kadar yakınsadığını kontrol et
            int closeParticles = 0;
            foreach (var particle in particles)
            {
                if (Math.Abs(particle.Position - globalBestPosition) < CONVERGENCE_THRESHOLD)
                {
                    closeParticles++;
                }
            }

            // Parçacıkların çoğu yakınsadıysa
            return closeParticles > (particleCount * 0.8);
        }

        private void ReinitializeParticles()
        {
            // En iyi pozisyonu koru ama parçacıkları yeniden dağıt
            double bestPos = globalBestPosition;
            double bestFit = globalBestFitness;
            InitializeParticles();

            // En iyi sonucu geri yükle
            globalBestPosition = bestPos;
            globalBestFitness = bestFit;

            // Arama parametrelerini sıfırla
            searchRadius = 360.0;
            inertiaWeight = 0.9;
            stagnationCounter = 0;
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
