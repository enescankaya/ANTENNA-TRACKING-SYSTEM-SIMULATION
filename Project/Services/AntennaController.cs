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
        private const double SIGNAL_THRESHOLD = 30.0; // Minimum kabul edilebilir sinyal gücü
        private const double MAX_VERTICAL_ANGLE = 90.0;
        private const double SCAN_STEP = 5.0; // Her adımdaki tarama açısı
        private bool isWideScanning = true;
        private double scanAreaSize = 360.0; // Başlangıçta tam daire tarama
        private const double MIN_SCAN_AREA = 30.0; // Minimum tarama alanı

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
            double rawSignal = CalculateSignalStrength(antenna, airplane);
            double filteredSignal = signalFilter.Update(rawSignal);
            antenna.SignalStrength = filteredSignal;
            antenna.RSSI = CalculateRSSI(antenna, airplane);
            antenna.SNR = CalculateSNR(antenna, airplane);

            if (filteredSignal < SIGNAL_THRESHOLD || isWideScanning)
            {
                PerformWideScan(antenna);
            }
            else
            {
                // PSO ile optimize edilmiş açıyı al
                double optimizedAngle = pso.GetNextAngle(filteredSignal);

                // Tarama alanını sinyal gücüne göre daralt
                if (filteredSignal > SIGNAL_THRESHOLD * 1.2)
                {
                    scanAreaSize = Math.Max(MIN_SCAN_AREA, scanAreaSize * 0.95);
                }

                // Kalman filtresi ile yumuşatılmış açılar
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
            // Yatay tarama
            double nextHAngle = (antenna.HorizontalAngle + SCAN_STEP) % 360;
            antenna.HorizontalAngle = horizontalFilter.Update(nextHAngle);

            // Dikey tarama
            if (nextHAngle < SCAN_STEP)
            {
                double nextVAngle = antenna.VerticalAngle + SCAN_STEP;
                if (nextVAngle > MAX_VERTICAL_ANGLE)
                {
                    nextVAngle = 0;
                    isWideScanning = antenna.SignalStrength < SIGNAL_THRESHOLD;
                    scanAreaSize = 360.0; // Reset scan area
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
            // Hedef açıları hesapla
            double targetHAngle = CalculateHorizontalAngle(airplane);
            double targetVAngle = CalculateVerticalAngle(airplane);

            // Açı farkları
            double hAngleDiff = Math.Abs(antenna.HorizontalAngle - targetHAngle);
            hAngleDiff = Math.Min(hAngleDiff, 360 - hAngleDiff); // En kısa açı farkı
            double vAngleDiff = Math.Abs(antenna.VerticalAngle - targetVAngle);

            // 3D mesafe hesaplama (normalize edilmiş koordinatlar)
            double distance = Math.Sqrt(
                Math.Pow(airplane.Latitude * 0.0001, 2) +
                Math.Pow(airplane.Longitude * 0.0001, 2) +
                Math.Pow(airplane.Altitude * 0.01, 2)
            );

            // Açı bazlı zayıflama
            double angleAttenuation = Math.Max(0, 1.0 - (hAngleDiff / 180.0) - (vAngleDiff / 90.0));

            // Mesafe bazlı zayıflama (ters kare kanunu)
            double distanceAttenuation = 1.0 / (1.0 + distance * distance);

            // Toplam sinyal gücü (0-100 arası)
            double signalStrength = (angleAttenuation * 0.7 + distanceAttenuation * 0.3) * 100;

            // Gürültü ekleme
            Random random = new Random();
            signalStrength += (random.NextDouble() - 0.5) * 5; // ±2.5 gürültü

            return Math.Max(0, Math.Min(100, signalStrength));
        }

        private double CalculateRSSI(AntennaState antenna, AirplaneState airplane)
        {
            // Mesafe bazlı RSSI hesaplama
            double distance = CalculateDistance(antenna, airplane);
            double maxDistance = 1000.0; // metre
            double minRSSI = -100.0; // dBm
            double maxRSSI = -40.0;  // dBm

            return minRSSI + (maxRSSI - minRSSI) * (1 - Math.Min(distance / maxDistance, 1));
        }

        private double CalculateSNR(AntennaState antenna, AirplaneState airplane)
        {
            // Açı ve mesafe bazlı SNR hesaplama
            double angleDiff = Math.Abs(antenna.HorizontalAngle - CalculateHorizontalAngle(airplane));
            double distance = CalculateDistance(antenna, airplane);

            double angleComponent = Math.Max(0, 1 - (angleDiff / 180.0));
            double distanceComponent = Math.Max(0, 1 - (distance / 1000.0));

            return (angleComponent * 0.7 + distanceComponent * 0.3) * 30.0; // 0-30 dB arası
        }

        private double CalculateDistance(AntennaState antenna, AirplaneState airplane)
        {
            return Math.Sqrt(
                Math.Pow((airplane.Latitude - antenna.Latitude) * 111000, 2) +
                Math.Pow((airplane.Longitude - antenna.Longitude) * 111000 * Math.Cos(antenna.Latitude * Math.PI / 180), 2) +
                Math.Pow(airplane.Altitude, 2)
            );
        }

        public void Reset()
        {
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
            isWideScanning = true;
            scanAreaSize = 360.0; // Reset scan area
            // PSO'yu yeniden başlat
            pso.GetNextAngle(0); // Reset için dummy call
        }
    }
}
