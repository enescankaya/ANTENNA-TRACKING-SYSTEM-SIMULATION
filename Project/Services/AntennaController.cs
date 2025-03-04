using System;
using Project.Models;

namespace Project.Services
{
    public class AntennaController
    {
        private readonly KalmanFilter horizontalFilter;
        private readonly KalmanFilter verticalFilter;
        private readonly KalmanFilter signalFilter;
        private readonly ParticleSwarmOptimizer pso;
        private const double SIGNAL_THRESHOLD = 20.0;
        private const double MAX_VERTICAL_ANGLE = 90.0;

        public AntennaController()
        {
            // Yatay açı için Kalman filtresi (daha hızlı tepki)
            horizontalFilter = new KalmanFilter(
                processNoise: 0.01,    // Daha yüksek process noise
                measurementNoise: 0.1,  // Normal measurement noise
                dt: 0.05               // Daha hızlı güncelleme
            );

            // Dikey açı için Kalman filtresi (daha stabil)
            verticalFilter = new KalmanFilter(
                processNoise: 0.005,    // Daha düşük process noise
                measurementNoise: 0.1,   // Normal measurement noise
                dt: 0.1                 // Normal güncelleme
            );

            // Sinyal için Kalman filtresi (daha fazla filtreleme)
            signalFilter = new KalmanFilter(
                processNoise: 0.001,    // Düşük process noise
                measurementNoise: 0.5,   // Yüksek measurement noise
                dt: 0.1                 // Normal güncelleme
            );

            pso = new ParticleSwarmOptimizer();
        }

        public void UpdateScanningAntenna(AntennaState antenna, AirplaneState airplane)
        {
            // Signal strength filtrelemesi
            double filteredSignal = signalFilter.Update(antenna.SignalStrength);
            antenna.SignalStrength = filteredSignal;

            if (filteredSignal < SIGNAL_THRESHOLD)
            {
                // Geniş tarama modu
                PerformWideScan(antenna);
            }
            else
            {
                // PSO optimizasyonu ve Kalman filtresi ile hassas takip
                double optimizedAngle = pso.GetNextAngle(filteredSignal);
                antenna.HorizontalAngle = horizontalFilter.Update(optimizedAngle);
                antenna.VerticalAngle = verticalFilter.Update(CalculateVerticalAngle(airplane));
            }
        }

        public void UpdateDirectionalAntenna(AntennaState antenna, AirplaneState airplane)
        {
            // Hedef açıları hesapla
            double targetHAngle = CalculateHorizontalAngle(airplane);
            double targetVAngle = CalculateVerticalAngle(airplane);

            // Kalman filtresi ile yumuşatılmış açılar
            antenna.HorizontalAngle = horizontalFilter.Update(targetHAngle);
            antenna.VerticalAngle = verticalFilter.Update(targetVAngle);

            // Sinyal gücü filtrelemesi
            antenna.SignalStrength = signalFilter.Update(CalculateSignalStrength(antenna, airplane));
        }

        private void PerformWideScan(AntennaState antenna)
        {
            const double SCAN_STEP = 5.0;

            // Yatay tarama
            double nextHAngle = (antenna.HorizontalAngle + SCAN_STEP) % 360;
            antenna.HorizontalAngle = horizontalFilter.Update(nextHAngle);

            // Dikey tarama (yatay tarama tamamlandığında)
            if (nextHAngle < SCAN_STEP)
            {
                double nextVAngle = antenna.VerticalAngle + SCAN_STEP;
                if (nextVAngle > MAX_VERTICAL_ANGLE)
                {
                    nextVAngle = 0; // Dikey taramayı sıfırla
                }
                antenna.VerticalAngle = verticalFilter.Update(nextVAngle);
            }
        }

        private double CalculateHorizontalAngle(AirplaneState airplane)
        {
            return (Math.Atan2(airplane.Longitude, airplane.Latitude) * 180 / Math.PI + 360) % 360;
        }

        private double CalculateVerticalAngle(AirplaneState airplane)
        {
            double distance = Math.Sqrt(
                airplane.Latitude * airplane.Latitude +
                airplane.Longitude * airplane.Longitude
            );
            return Math.Atan2(airplane.Altitude, distance) * 180 / Math.PI;
        }

        private double CalculateSignalStrength(AntennaState antenna, AirplaneState airplane)
        {
            // Mesafe ve açı farkına göre sinyal gücü hesaplama
            double distance = Math.Sqrt(
                airplane.Latitude * airplane.Latitude +
                airplane.Longitude * airplane.Longitude +
                airplane.Altitude * airplane.Altitude
            );

            double angleDiff = Math.Abs(antenna.HorizontalAngle - CalculateHorizontalAngle(airplane));
            angleDiff = Math.Min(angleDiff, 360 - angleDiff);

            // Normalize edilmiş sinyal gücü (0-100 arası)
            return Math.Max(0, 100 - (distance * 0.1) - (angleDiff * 0.5));
        }

        public void Reset()
        {
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
        }
    }
}
