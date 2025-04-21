using System;
using System.Collections.Generic;
using Project.Models;

namespace Project.Services
{
    public class AntennaController
    {
        private readonly KalmanFilter horizontalFilter;
        private readonly KalmanFilter verticalFilter;
        private readonly KalmanFilter signalFilter;
        private readonly ParticleSwarmOptimizer pso;

        private const double MIN_SCAN_AREA = 10.0;
        private const double MAX_SCAN_AREA = 360.0;
        private const double INITIAL_SCAN_DURATION = 10.0; // Tarama süresini artırdık
        private const double SIGNAL_LOSS_THRESHOLD = 15.0;
        private const double TRACKING_THRESHOLD = 40.0;

        // Hedef pozisyon ve sinyal ağırlıkları
        private const double TARGET_WEIGHT = 0.8; // Hedef pozisyonun önemini artırdık
        private const double SIGNAL_WEIGHT = 0.2;

        private bool isInitialScan = true;
        private DateTime scanStartTime;
        private double bestSignalStrength = 0;
        private double bestHorizontalAngle = 0;
        private double bestVerticalAngle = 0;
        private double lastKnownTargetH = 0;
        private double lastKnownTargetV = 0;

        // Tarama parametreleri
        private int currentGridStepH = 0;
        private int currentGridStepV = 0;
        private const int GRID_STEPS_H = 36; // Daha fazla adım, daha iyi kapsama
        private const int GRID_STEPS_V = 18;
        private DateTime lastScanUpdate = DateTime.MinValue;
        private const double SCAN_UPDATE_RATE = 0.02;

        // Sinyal takibi için değişkenler
        private double signalQualityHistory = 0;
        private const double SIGNAL_HISTORY_WEIGHT = 0.7;

        // PSO güncellemesi için değişkenler
        private DateTime lastPsoUpdate = DateTime.MinValue;
        private const double PSO_UPDATE_RATE = 0.08;

        // Hedef takibi için değişkenler
        private double predictedHorizontalAngle = 0;
        private double predictedVerticalAngle = 0;
        private double horizontalVelocity = 0;
        private double verticalVelocity = 0;
        private DateTime lastPositionUpdate = DateTime.MinValue;

        // Uçak konum geçmişi
        private Queue<(double h, double v, DateTime time)> positionHistory = new Queue<(double, double, DateTime)>();
        private const int MAX_HISTORY_POINTS = 5;

        // Public properties
        public bool IsInitialScan => isInitialScan;
        public double CurrentScanArea { get; private set; } = MAX_SCAN_AREA;
        public double ScanProgress => isInitialScan ?
            Math.Min(1.0, (DateTime.Now - scanStartTime).TotalSeconds / INITIAL_SCAN_DURATION) : 1.0;
        public int ParticleCount => pso?.ParticleCount ?? 0;
        public double BestSignalStrength => bestSignalStrength;
        public double TargetHorizontalAngle => lastKnownTargetH;
        public double TargetVerticalAngle => lastKnownTargetV;

        public AntennaController()
        {
            // Kalman filtrelerini başlat
            horizontalFilter = new KalmanFilter(0.1, 0.3, 0.1);
            verticalFilter = new KalmanFilter(0.08, 0.3, 0.1);
            signalFilter = new KalmanFilter(0.05, 0.2, 0.1);

            // Daha fazla parçacıkla PSO başlat
            pso = new ParticleSwarmOptimizer(60);

            Reset();
        }

        public void UpdateScanningAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            // Güncelleme sıklığını sınırla
            if ((DateTime.Now - lastScanUpdate).TotalSeconds < SCAN_UPDATE_RATE) return;
            lastScanUpdate = DateTime.Now;

            double antennaLat = antenna.Latitude;
            double antennaLon = antenna.Longitude;
            double antennaAlt = antenna.Altitude;

            // Uçağa göre hedef açıları hesapla
            GetTargetAngles(antennaLat, antennaLon, antennaAlt, airplane, out double targetH, out double targetV);

            // Hedef açıları kaydet
            lastKnownTargetH = targetH;
            lastKnownTargetV = targetV;

            // Uçak konumunu takip et
            UpdatePositionHistory(targetH, targetV);

            // Uçağın hareketini tahmin et
            PredictTargetMovement();

            if (isInitialScan)
            {
                // İlk tarama modunda ise sistematik tarama yap
                PerformInitialScan(antenna, airplane, antennaLat, antennaLon, antennaAlt);
            }
            else
            {
                // Takip modunda ise PSO ile optimizasyon yap
                PerformTracking(antenna, airplane, antennaLat, antennaLon, antennaAlt, targetH, targetV);
            }

            // Sinyal metriklerini güncelle (RSSI, SNR)
            UpdateSignalMetrics(antenna);
        }

        private void UpdatePositionHistory(double targetH, double targetV)
        {
            // Konum geçmişini güncelle, uçağın hareketini takip etmek için
            positionHistory.Enqueue((targetH, targetV, DateTime.Now));

            // En fazla MAX_HISTORY_POINTS kadar konum tut
            if (positionHistory.Count > MAX_HISTORY_POINTS)
            {
                positionHistory.Dequeue();
            }
        }

        private void PredictTargetMovement()
        {
            // En az 2 konum varsa hız hesapla
            if (positionHistory.Count < 2) return;

            var positions = positionHistory.ToArray();
            var oldest = positions[0];
            var newest = positions[positions.Length - 1];

            double timeDiff = (newest.time - oldest.time).TotalSeconds;
            if (timeDiff < 0.1) return; // Çok kısa sürede değişim varsa hesaplama yapma

            // Yatay açı değişimini hesapla (0-360 geçişlerini dikkate al)
            double hDiff = newest.h - oldest.h;
            if (hDiff > 180) hDiff -= 360;
            if (hDiff < -180) hDiff += 360;

            // Dikey açı değişimini hesapla
            double vDiff = newest.v - oldest.v;

            // Açısal hızları hesapla
            horizontalVelocity = hDiff / timeDiff;
            verticalVelocity = vDiff / timeDiff;

            // Gelecek konumu tahmin et (0.5 saniye sonrası için)
            predictedHorizontalAngle = (newest.h + horizontalVelocity * 0.5 + 360) % 360;
            predictedVerticalAngle = Math.Max(0, Math.Min(90, newest.v + verticalVelocity * 0.5));

            lastPositionUpdate = DateTime.Now;
        }

        private void PerformInitialScan(AntennaState antenna, AirplaneState airplane,
                                      double antennaLat, double antennaLon, double antennaAlt)
        {
            // İlk tarama başlatma
            if (scanStartTime == default)
            {
                scanStartTime = DateTime.Now;
                currentGridStepH = 0;
                currentGridStepV = 0;
                bestSignalStrength = 0;
            }

            double elapsed = (DateTime.Now - scanStartTime).TotalSeconds;

            // Spiral tarama paterni - tüm 360 derecelik alanı kapsamak için
            double totalPoints = GRID_STEPS_H * GRID_STEPS_V;
            double scanProgress = Math.Min(1.0, elapsed / INITIAL_SCAN_DURATION);
            int currentPoint = (int)(scanProgress * totalPoints);

            // Izgarada ilerleme
            currentGridStepH = currentPoint % GRID_STEPS_H;
            currentGridStepV = (currentPoint / GRID_STEPS_H) % GRID_STEPS_V;

            // Tüm ızgarayı taradıysak taramayı bitir
            if (currentPoint >= totalPoints)
            {
                isInitialScan = false;
                pso.Reset();
                CurrentScanArea = Math.Max(MIN_SCAN_AREA, 180.0 * (1.0 - bestSignalStrength / 100.0));
                return;
            }

            // Izgaradaki mevcut konumu hesapla
            double gridH = 360.0 / GRID_STEPS_H;
            double gridV = 90.0 / GRID_STEPS_V;

            double scanH = currentGridStepH * gridH;
            double scanV = currentGridStepV * gridV;

            // Anten hareketlerini filtreleme ile uygula
            antenna.HorizontalAngle = horizontalFilter.Update(scanH);
            antenna.VerticalAngle = verticalFilter.Update(scanV);
            antenna.ScanAreaSize = MAX_SCAN_AREA;

            // Mevcut konumdaki sinyal gücünü hesapla
            double signal = CalculateSignalStrength(scanH, scanV, antennaLat, antennaLon, antennaAlt, airplane);
            signal = signalFilter.Update(signal);
            antenna.SignalStrength = signal;

            // En iyi sinyal konumunu güncelle
            if (signal > bestSignalStrength)
            {
                bestSignalStrength = signal;
                bestHorizontalAngle = scanH;
                bestVerticalAngle = scanV;

                // Debug bilgisi
                Console.WriteLine($"Initial Scan: Found better signal: {signal} at H:{scanH:F1}, V:{scanV:F1}");
            }

            // Çok güçlü sinyal bulduysak erken çık
            if (signal > TRACKING_THRESHOLD * 1.2)
            {
                Console.WriteLine($"Initial Scan: Strong signal found ({signal}), switching to tracking mode");
                isInitialScan = false;
                pso.Reset();
                CurrentScanArea = Math.Max(MIN_SCAN_AREA, 180.0 * (1.0 - bestSignalStrength / 100.0));
            }
        }

        private void PerformTracking(AntennaState antenna, AirplaneState airplane,
                                   double antennaLat, double antennaLon, double antennaAlt,
                                   double targetH, double targetV)
        {
            if ((DateTime.Now - lastPsoUpdate).TotalSeconds >= PSO_UPDATE_RATE)
            {
                lastPsoUpdate = DateTime.Now;

                // Önce doğrudan hedefi takip et
                antenna.HorizontalAngle = horizontalFilter.Update(targetH);
                antenna.VerticalAngle = verticalFilter.Update(targetV);

                // Sinyal gücünü kontrol et
                double directSignal = CalculateSignalStrength(targetH, targetV, antennaLat, antennaLon, antennaAlt, airplane);

                // Eğer sinyal yeterince güçlüyse (>60) hedefi doğrudan takip et
                if (directSignal > 60)
                {
                    bestSignalStrength = directSignal;
                    bestHorizontalAngle = targetH;
                    bestVerticalAngle = targetV;
                    CurrentScanArea = Math.Max(MIN_SCAN_AREA, 20.0); // Çok dar tarama
                }
                // Değilse PSO ile optimize et
                else
                {
                    // Tarama alanını sinyal gücüne göre ayarla
                    double searchArea = directSignal > 40 ? 45.0 : 90.0;
                    CurrentScanArea = Math.Max(MIN_SCAN_AREA, searchArea);

                    // PSO'yu hedef etrafında ara
                    var (offsetH, offsetV) = pso.ScanStep(
                        (h, v) =>
                        {
                            double candidateH = (targetH + h + 360.0) % 360.0;
                            double candidateV = Math.Max(0, Math.Min(90.0, targetV + v));
                            return CalculateSignalStrength(candidateH, candidateV, antennaLat, antennaLon, antennaAlt, airplane);
                        },
                        CurrentScanArea,
                        Math.Min(30.0, CurrentScanArea / 2),
                        targetH,
                        targetV
                    );

                    // Hedefi merkez alarak yeni pozisyonu hesapla
                    double newH = (targetH + offsetH * 0.3 + 360.0) % 360.0;
                    double newV = Math.Max(0, Math.Min(90.0, targetV + offsetV * 0.3));

                    antenna.HorizontalAngle = horizontalFilter.Update(newH);
                    antenna.VerticalAngle = verticalFilter.Update(newV);
                }

                // Sinyal metriklerini güncelle
                double signal = CalculateSignalStrength(
                    antenna.HorizontalAngle,
                    antenna.VerticalAngle,
                    antennaLat, antennaLon, antennaAlt,
                    airplane
                );

                signal = signalFilter.Update(signal);
                antenna.SignalStrength = signal;

                if (signal > bestSignalStrength)
                {
                    bestSignalStrength = signal;
                    bestHorizontalAngle = antenna.HorizontalAngle;
                    bestVerticalAngle = antenna.VerticalAngle;
                }

                // Sinyal kaybı kontrolü
                signalQualityHistory = signal * 0.3 + signalQualityHistory * 0.7;
                if (signalQualityHistory < SIGNAL_LOSS_THRESHOLD)
                {
                    isInitialScan = true;
                    scanStartTime = DateTime.Now;
                    CurrentScanArea = MAX_SCAN_AREA;
                    bestSignalStrength = 0;
                }
            }
        }

        public void UpdateDirectionalAntenna(AntennaState antenna, AntennaState scanningAntenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            // Her zaman tarayıcı antenin bulduğu en iyi konuma yönlen
            antenna.HorizontalAngle = horizontalFilter.Update(bestHorizontalAngle);
            antenna.VerticalAngle = verticalFilter.Update(bestVerticalAngle);
            antenna.ScanAreaSize = 0; // Yönlü anten için tarama alanı yok

            double antennaLat = antenna.Latitude;
            double antennaLon = antenna.Longitude;
            double antennaAlt = antenna.Altitude;

            // Yönlü anten için sinyal metriklerini hesapla
            antenna.SignalStrength = CalculateSignalStrength(
                antenna.HorizontalAngle, antenna.VerticalAngle,
                antennaLat, antennaLon, antennaAlt, airplane);

            UpdateSignalMetrics(antenna);
        }

        private void GetTargetAngles(double antennaLat, double antennaLon, double antennaAlt,
                                   AirplaneState airplane, out double targetH, out double targetV)
        {
            // Mesafeyi metre cinsinden hesapla
            double dLat = (airplane.Latitude - antennaLat) * 111000;
            double dLon = (airplane.Longitude - antennaLon) * 111000 * Math.Cos(antennaLat * Math.PI / 180.0);
            double dAlt = airplane.Altitude - antennaAlt;

            // Yerdeki mesafe ve açıları hesapla
            double groundDist = Math.Sqrt(dLat * dLat + dLon * dLon);

            // Yatay açı (azimut)
            targetH = (Math.Atan2(dLon, dLat) * 180.0 / Math.PI + 360.0) % 360.0;

            // Dikey açı (yükseklik)
            targetV = Math.Atan2(dAlt, groundDist) * 180.0 / Math.PI;
            targetV = Math.Max(0, Math.Min(90, targetV)); // 0-90 derece aralığına sınırla
        }

        private double CalculateSignalStrength(double antennaH, double antennaV,
                                             double antennaLat, double antennaLon, double antennaAlt,
                                             AirplaneState airplane)
        {
            // Gerçek hedef açılarını hesapla
            double dLat = (airplane.Latitude - antennaLat) * 111000;
            double dLon = (airplane.Longitude - antennaLon) * 111000 * Math.Cos(antennaLat * Math.PI / 180.0);
            double dAlt = airplane.Altitude - antennaAlt;

            double groundDist = Math.Sqrt(dLat * dLat + dLon * dLon);
            double distance = Math.Sqrt(groundDist * groundDist + dAlt * dAlt);

            double targetH = (Math.Atan2(dLon, dLat) * 180.0 / Math.PI + 360.0) % 360.0;
            double targetV = Math.Atan2(dAlt, groundDist) * 180.0 / Math.PI;
            targetV = Math.Max(0, Math.Min(90, targetV)); // 0-90 derece aralığına sınırla

            // Açı farklarını doğru şekilde hesapla (0-360 geçişlerini dikkate al)
            double hAngleDiff = Math.Min(
                Math.Min(
                    Math.Abs(antennaH - targetH),
                    Math.Abs(antennaH - targetH + 360)
                ),
                Math.Abs(antennaH - targetH - 360)
            );
            double vAngleDiff = Math.Abs(antennaV - targetV);

            // Daha gerçekçi anten kazanç modeli (daha dar ışın deseni)
            double hGain = Math.Exp(-Math.Pow(hAngleDiff / 12.0, 2));
            double vGain = Math.Exp(-Math.Pow(vAngleDiff / 6.0, 2));

            // Mesafe zayıflaması (ters kare yasası)
            double distAtt = 15000.0 / (15000.0 + distance);

            // Son sinyal gücü için faktörleri birleştir
            double signal = 100.0 * hGain * vGain * distAtt;

            // Gerçek dünya koşullarını simüle etmek için rasgele gürültü ekle
            double noise = (new Random().NextDouble() - 0.5) * 2.0;

            return Math.Max(0, Math.Min(100, signal + noise));
        }

        private void UpdateSignalMetrics(AntennaState antenna)
        {
            // RSSI (Alınan Sinyal Gücü Göstergesi) hesapla
            // RSSI tipik olarak -100 dBm (zayıf) ile -30 dBm (güçlü) arasında değişir
            double signalPercent = antenna.SignalStrength / 100.0;
            double baseRSSI = -110; // Minimum RSSI değeri
            double maxRSSI = -30;   // Maksimum RSSI değeri

            // Doğrusal olmayan eşleme ile daha gerçekçi davranış
            antenna.RSSI = baseRSSI + (maxRSSI - baseRSSI) * Math.Pow(signalPercent, 0.6);

            // SNR (Sinyal-Gürültü Oranı) hesapla
            // SNR tipik olarak 0 dB (kötü) ile 40+ dB (mükemmel) arasında değişir
            double baseSNR = 0;    // Minimum SNR değeri
            double maxSNR = 45;    // Maksimum SNR değeri

            // Doğrusal olmayan eşleme ile daha gerçekçi davranış
            antenna.SNR = baseSNR + (maxSNR - baseSNR) * Math.Pow(signalPercent, 0.8);
        }

        // Tüm durumu sıfırla
        public void Reset()
        {
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
            pso.Reset();
            isInitialScan = true;
            scanStartTime = default;
            bestSignalStrength = 0;
            signalQualityHistory = 0;
            CurrentScanArea = MAX_SCAN_AREA;
            positionHistory.Clear();
            predictedHorizontalAngle = 0;
            predictedVerticalAngle = 0;
            horizontalVelocity = 0;
            verticalVelocity = 0;
        }
    }
}