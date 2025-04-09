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
        private const double SIGNAL_THRESHOLD = 30.0; // Eşik değeri yükseltildi
        private const double MAX_VERTICAL_ANGLE = 90.0;
        private bool isWideScanning = true; // Geniş tarama modunu takip etmek için

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
            double rawSignal = CalculateSignalStrength(antenna, airplane);
            double filteredSignal = signalFilter.Update(rawSignal);
            antenna.SignalStrength = filteredSignal;

            if (filteredSignal < SIGNAL_THRESHOLD || isWideScanning)
            {
                // Geniş tarama modu
                PerformWideScan(antenna);
            }
            else
            {
                // PSO optimizasyonu ve Kalman filtresi ile hassas takip
                double optimizedAngle = pso.GetNextAngle(filteredSignal);

                // Yatay açıyı filtrele ve uygula
                antenna.HorizontalAngle = horizontalFilter.Update(optimizedAngle);

                // Dikey açıyı hesapla ve filtrele
                double targetVAngle = CalculateVerticalAngle(airplane);
                antenna.VerticalAngle = verticalFilter.Update(targetVAngle);

                // Signal strength yeterince iyiyse geniş taramayı kapat
                if (filteredSignal > SIGNAL_THRESHOLD * 1.2) // %20 marj
                {
                    isWideScanning = false;
                }
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
                    isWideScanning = true; // Yeni bir tarama döngüsü başlat
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

        public void Reset()
        {
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
            isWideScanning = true;
            // PSO'yu yeniden başlat
            pso.GetNextAngle(0); // Reset için dummy call
        }
    }
}
