using System;
using System.Collections.Generic;
using System.Linq;
using Project.Models;

namespace Project.Services
{
    /// <summary>
    /// Enhanced antenna controller that uses Particle Swarm Optimization to efficiently track aircraft
    /// </summary>
    public class AntennaController
    {
        private readonly KalmanFilter horizontalFilter;
        private readonly KalmanFilter verticalFilter;
        private readonly KalmanFilter signalFilter;
        private ParticleSwarmOptimizer pso;

        // Scan area parameters - değişkenleri güncelle
        private const double INITIAL_SCAN_AREA_H = 360.0;
        private const double INITIAL_SCAN_AREA_V = 90.0;
        private const double MIN_SCAN_AREA_H = 15.0;       // Daha geniş minimum tarama (önceki: 10.0)
        private const double MIN_SCAN_AREA_V = 7.5;        // Daha geniş minimum tarama (önceki: 5.0)
        private const double MAX_SCAN_AREA_H = INITIAL_SCAN_AREA_H; // Maximum değeri başlangıç değerine eşitle
        private const double MAX_SCAN_AREA_V = INITIAL_SCAN_AREA_V; // Maximum değeri başlangıç değerine eşitle
        private const double SCAN_AREA_REDUCTION_RATE = 0.95; // Her adımda %5 küçült

        // Sinyal eşikleri - kademeli küçültme için
        private const double EXCELLENT_SIGNAL = 80.0;
        private const double GOOD_SIGNAL = 60.0;
        private const double FAIR_SIGNAL = 40.0;
        private const double POOR_SIGNAL = 20.0;

        // Initial scan parameters - Tarama süresini artır
        private const double INITIAL_SCAN_DURATION = 30.0; // 30 saniyeye çıkarıldı
        private const double TRACKING_THRESHOLD = 40.0;    // Daha düşük eşik (önceki: 50.0)
        private const double SIGNAL_LOSS_THRESHOLD = 15.0; // Daha düşük eşik (önceki: 20.0)

        // Target weighting factors
        private const double TARGET_WEIGHT = 0.7;      // How much to favor the calculated target position
        private const double SIGNAL_WEIGHT = 0.3;     // How much to favor the best signal position
        private const double PREDICTION_WEIGHT = 0.3; // How much to use predictive tracking

        // Scan state variables
        private bool isInitialScan = true;
        private DateTime scanStartTime;
        private double bestSignalStrength = 0;
        private double bestHorizontalAngle = 0;
        private double bestVerticalAngle = 0;
        private double lastKnownTargetH = 0;
        private double lastKnownTargetV = 0;

        // Grid scan parameters
        private int currentGridStepH = 0;
        private int currentGridStepV = 0;
        private const int GRID_STEPS_H = 36; // 10-degree horizontal steps
        private const int GRID_STEPS_V = 18; // 5-degree vertical steps

        // Initial scan grid for 360x90 coverage
        private List<(double h, double v)> initialScanGrid = new List<(double h, double v)>();
        private int initialScanIndex = 0;
        private const int INITIAL_SCAN_POINTS = 400; // ~20x20 grid, adjust for density

        // Update rate variables - Güncelleme hızını yavaşlat
        private DateTime lastScanUpdate = DateTime.MinValue;
        private const double SCAN_UPDATE_RATE = 0.05;  // 20Hz'e düşür
        private DateTime lastPsoUpdate = DateTime.MinValue;
        private const double PSO_UPDATE_RATE = 0.05;    // 10Hz'e düşür

        // Signal quality tracking
        private double signalQualityHistory = 0;
        private const double SIGNAL_HISTORY_WEIGHT = 0.7;

        // Aircraft movement prediction
        private double predictedHorizontalAngle = 0;
        private double predictedVerticalAngle = 0;
        private double horizontalVelocity = 0;
        private double verticalVelocity = 0;
        private DateTime lastPositionUpdate = DateTime.MinValue;

        // Aircraft position history for movement prediction
        private Queue<(double h, double v, DateTime time)> positionHistory = new Queue<(double, double, DateTime)>();
        private const int MAX_HISTORY_POINTS = 10;

        // Debugging and visualization properties
        private double horizontalScanArea = INITIAL_SCAN_AREA_H;
        private double verticalScanArea = INITIAL_SCAN_AREA_V;
        private int scanPhaseDegrees = 0;
        private bool targetLocked = false;
        private string currentScanMode = "Initial Scan";

        // Exploration fazı için ek değişkenler
        private bool isExplorationPhase = true;
        private double explorationElapsed = 0;
        private const double EXPLORATION_DURATION = 8.0; // saniye, ilk fazda tüm alanı tarama süresi
        private double explorationAngle = 0;
        private double explorationAngleStep = 10.0; // Her adımda 10 derece kaydır

        // PSO reset parameters
        private const int MAX_PSO_ITERATIONS = 50;
        private const int PARTICLE_COUNT = 30;
        private DateTime lastPsoReset = DateTime.MinValue;
        private const double PSO_RESET_INTERVAL = 3.0; // 2 seconds

        // Signal strength calculation parameters
        private const double MAX_RSSI = -40.0;  // dBm, very close to transmitter
        private const double MIN_RSSI = -120.0; // dBm, at maximum range
        private const double REFERENCE_DISTANCE = 1000.0; // meters
        private const double PATH_LOSS_EXPONENT = 2.0;   // Free space path loss

        // Sabitler - Ani sapmaları önlemek için
        private const double MAX_ANGLE_CHANGE = 30.0;  // Bir adımda maksimum açı değişimi
        private const double SIGNAL_STABILITY_THRESHOLD = 5.0; // Sinyal kararlılık eşiği
        private const int STABLE_SIGNAL_COUNT = 3; // Kararlı sinyal sayısı
        private readonly Queue<double> recentSignals = new Queue<double>();
        private double lastValidHorizontalAngle = 0;
        private double lastValidVerticalAngle = 0;
        private bool hasValidPosition = false;

        // Public properties for UI display and monitoring
        public bool IsInitialScan => isInitialScan;
        public double CurrentScanArea => Math.Max(horizontalScanArea, verticalScanArea);
        public double CurrentHorizontalScanArea => horizontalScanArea;
        public double CurrentVerticalScanArea => verticalScanArea;
        public double ScanProgress => isInitialScan ?
            Math.Min(1.0, (DateTime.Now - scanStartTime).TotalSeconds / INITIAL_SCAN_DURATION) : 1.0;
        public int ParticleCount => pso?.ParticleCount ?? 0;
        public double BestSignalStrength => bestSignalStrength;
        public double TargetHorizontalAngle => lastKnownTargetH;
        public double TargetVerticalAngle => lastKnownTargetV;
        public double PredictedHorizontalAngle => predictedHorizontalAngle;
        public double PredictedVerticalAngle => predictedVerticalAngle;
        public int ScanPhase => scanPhaseDegrees;
        public bool IsTargetLocked => targetLocked;
        public string CurrentMode => currentScanMode;
        public double ConvergenceRate => pso?.ConvergenceRate ?? 0;
        public double SearchRadius => pso?.SearchRadius ?? 180.0;
        public int PsoIteration => pso?.CurrentIteration ?? 0;
        public double HorizontalVelocity => horizontalVelocity;
        public double VerticalVelocity => verticalVelocity;

        public AntennaController()
        {
            // Düzeltme: Kalman filtre parametrelerini daha hızlı tepki için ayarla
            horizontalFilter = new KalmanFilter(0.25, 0.3, 0.5);  // Daha hızlı tepki (önceki: 0.05, 0.5, 0.2)
            verticalFilter = new KalmanFilter(0.25, 0.3, 0.5);    // Daha hızlı tepki 
            signalFilter = new KalmanFilter(0.3, 0.2, 0.3);       // Sinyal filtresini daha da hassaslaştır

            pso = new ParticleSwarmOptimizer(100); // Parçacık sayısı yeterli
            Reset();
        }

        /// <summary>
        /// Updates the scanning antenna's position to search for and track aircraft
        /// </summary>
        public double UpdateScanningAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return MIN_RSSI;

            // Check if it's time to reset PSO
            if ((DateTime.Now - lastPsoReset).TotalSeconds >= PSO_RESET_INTERVAL)
            {
                pso.Reset();
                pso = new ParticleSwarmOptimizer(PARTICLE_COUNT);
                lastPsoReset = DateTime.Now;
                System.Diagnostics.Debug.WriteLine("PSO Reset performed");
            }

            // Düzeltme: Güncelleme hızını artır
            if ((DateTime.Now - lastScanUpdate).TotalSeconds < SCAN_UPDATE_RATE * 0.5) return MIN_RSSI;
            lastScanUpdate = DateTime.Now;

            double antennaLat = antenna.Latitude;
            double antennaLon = antenna.Longitude;
            double antennaAlt = antenna.Altitude;

            // Her durumda güncel hedef açılarını al
            GetTargetAngles(antennaLat, antennaLon, antennaAlt, airplane, out double targetH, out double targetV);

            // Gerçek hedef açılarını güncelle
            lastKnownTargetH = targetH;
            lastKnownTargetV = targetV;

            // Düzeltme: Acquisition ve Tracking arasındaki geçişi kolaylaştır
            if (!isInitialScan && !isExplorationPhase)
            {
                // Gerçek sinyal gücünü ölç
                double realSignal = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);

                // Sinyal eşiklerini kontrol et
                if (realSignal > TRACKING_THRESHOLD * 0.85 && !targetLocked)
                {
                    // Signal Acquisition -> Target Tracking geçişini kolaylaştır
                    targetLocked = true;
                    currentScanMode = "Target Tracking";
                    bestSignalStrength = realSignal;
                }
                else if (realSignal < SIGNAL_LOSS_THRESHOLD * 1.2 && targetLocked)
                {
                    // Target Tracking -> Signal Acquisition geçişini hızlandır
                    targetLocked = false;
                    currentScanMode = "Signal Re-Acquisition";
                    bestSignalStrength = realSignal;

                    // Tarama alanını genişlet
                    horizontalScanArea = Math.Min(MAX_SCAN_AREA_H, 60.0);
                    verticalScanArea = Math.Min(MAX_SCAN_AREA_V, 30.0);
                }
            }
            // 1. Exploration fazı: PSO yok, sistematik sweep ile tüm alanı gez
            if (isExplorationPhase)
            {
                currentScanMode = "Exploration Sweep";
                explorationElapsed += SCAN_UPDATE_RATE;
                // Sweep açısını ilerlet
                explorationAngle = (explorationAngle + explorationAngleStep) % 360.0;
                double sweepV = 45.0 + 40.0 * Math.Sin((explorationAngle / 360.0) * Math.PI); // 10-80 derece arası yukarı-aşağı
                antenna.HorizontalAngle = horizontalFilter.Update(explorationAngle);
                antenna.VerticalAngle = verticalFilter.Update(sweepV);

                // Sinyal ölçümü
                double signal = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);
                signal = signalFilter.Update(signal);
                antenna.SignalStrength = signal;

                // En iyi sinyali kaydet
                if (signal > bestSignalStrength)
                {
                    bestSignalStrength = signal;
                    bestHorizontalAngle = antenna.HorizontalAngle;
                    bestVerticalAngle = antenna.VerticalAngle;
                }

                // Exploration süresi dolduysa veya güçlü sinyal bulduysa PSO fazına geç
                if (explorationElapsed >= EXPLORATION_DURATION || bestSignalStrength > 30)
                {
                    isExplorationPhase = false;
                    isInitialScan = true;
                    scanStartTime = DateTime.Now;
                    // PSO arama alanını geniş başlat
                    horizontalScanArea = INITIAL_SCAN_AREA_H;
                    verticalScanArea = INITIAL_SCAN_AREA_V;
                }
                return MIN_RSSI;
            }
            else if (isInitialScan)
            {
                currentScanMode = "Initial 360° Scan";
                PerformInitialScan(antenna, airplane, antennaLat, antennaLon, antennaAlt);
            }
            else if (bestSignalStrength < SIGNAL_LOSS_THRESHOLD)
            {
                currentScanMode = "Wide Area Search";
                targetLocked = false;
                PerformWideAreaSearch(antenna, airplane, antennaLat, antennaLon, antennaAlt, lastKnownTargetH, lastKnownTargetV);
            }
            else if (!targetLocked || bestSignalStrength < TRACKING_THRESHOLD)
            {
                currentScanMode = "Signal Acquisition";
                targetLocked = false;
                PerformSignalAcquisition(antenna, airplane, antennaLat, antennaLon, antennaAlt, lastKnownTargetH, lastKnownTargetV);
            }
            else
            {
                currentScanMode = "Target Tracking";
                PerformTargetTracking(antenna, airplane, antennaLat, antennaLon, antennaAlt, lastKnownTargetH, lastKnownTargetV);
            }

            // Sinyal metriklerini güncelle...
            UpdateSignalMetrics(antenna);
            scanPhaseDegrees = (int)(antenna.HorizontalAngle % 360);

            // Calculate realistic RSSI based on distance
            double distance = CalculateDistance(
                antenna.Latitude, antenna.Longitude, antenna.Altitude,
                airplane.Latitude, airplane.Longitude, airplane.Altitude
            );

            // Calculate path loss
            double pathLoss = CalculatePathLoss(distance);

            // Calculate angle loss
            double angleLoss = CalculateAngleLoss(
                antenna.HorizontalAngle, antenna.VerticalAngle,
                lastKnownTargetH, lastKnownTargetV
            );

            // Final RSSI calculation
            double rssi = MAX_RSSI - pathLoss - angleLoss;
            rssi = Math.Max(MIN_RSSI, Math.Min(MAX_RSSI, rssi));

            // Update antenna metrics
            antenna.RSSI = rssi;
            antenna.SNR = CalculateSNR(rssi);
            antenna.SignalStrength = ConvertRssiToSignalStrength(rssi);

            return rssi;
        }

        private double CalculatePathLoss(double distance)
        {
            // Free space path loss calculation
            double pathLoss = 20 * Math.Log10(distance / REFERENCE_DISTANCE);
            pathLoss *= PATH_LOSS_EXPONENT;

            // Add random variations for realism
            Random rand = new Random();
            double variation = (rand.NextDouble() - 0.5) * 2; // ±1 dB variation

            return pathLoss + variation;
        }

        private double CalculateAngleLoss(double currentH, double currentV, double targetH, double targetV)
        {
            // Calculate angular difference
            double hDiff = Math.Abs(((targetH - currentH + 540.0) % 360.0) - 180.0);
            double vDiff = Math.Abs(targetV - currentV);

            // Convert angular difference to loss
            // 3dB loss at 30 degrees off-axis
            double hLoss = Math.Pow(hDiff / 30.0, 2) * 3.0;
            double vLoss = Math.Pow(vDiff / 30.0, 2) * 3.0;

            return hLoss + vLoss;
        }

        private double CalculateSNR(double rssi)
        {
            const double NOISE_FLOOR = -120; // dBm
            double snr = rssi - NOISE_FLOOR;
            return Math.Max(0, snr);
        }

        private double ConvertRssiToSignalStrength(double rssi)
        {
            // Convert RSSI (-120 to -40 dBm) to signal strength (0-100%)
            double normalizedRssi = (rssi - MIN_RSSI) / (MAX_RSSI - MIN_RSSI);
            return Math.Max(0, Math.Min(100, normalizedRssi * 100));
        }

        // SimulateSignalStrength fonksiyonunu gerçekçi hale getir
        private double SimulateSignalStrength(AntennaState antenna, AirplaneState airplane,
                                           double antennaLat, double antennaLon, double antennaAlt)
        {
            // Gerçek hedef açılarını hesapla - bu önemli!
            GetTargetAngles(antennaLat, antennaLon, antennaAlt, airplane, out double targetH, out double targetV);

            // Düzeltme: Hedef açılarını her zaman güncelle - bu çok önemli!
            lastKnownTargetH = targetH;
            lastKnownTargetV = targetV;

            // Gerçek mesafe
            double distance = CalculateDistance(antennaLat, antennaLon, antennaAlt,
                                             airplane.Latitude, airplane.Longitude, airplane.Altitude);

            // Açısal sapma
            double hDiff = Math.Abs(((targetH - antenna.HorizontalAngle + 540.0) % 360.0) - 180.0);
            double vDiff = Math.Abs(targetV - antenna.VerticalAngle);

            // Mesafe bazlı zayıflama (logaritmik, gerçekçi)
            double maxRange = 100000.0; // 100km
            double minRange = 100.0;    // 100m
            double normDist = Math.Min(1.0, Math.Max(0, (distance - minRange) / (maxRange - minRange)));
            double distanceAttenuation = 1.0 - normDist; // Uzakta 0, yakında 1

            // Açısal zayıflama (daha keskin anten paterni)
            double beamAngleH = 15.0;
            double beamAngleV = 10.0;
            double hGain = Math.Exp(-1.0 * Math.Pow(hDiff / (beamAngleH * 0.5), 2));
            double vGain = Math.Exp(-1.0 * Math.Pow(vDiff / (beamAngleV * 0.5), 2));

            // Toplam sinyal gücü
            double baseSignal = 100.0;
            double totalGain = hGain * vGain * distanceAttenuation;
            double signalStrength = baseSignal * totalGain;

            // SNR ve RSSI
            double noiseFloor = -90.0;
            double receivedPower = signalStrength - 100.0;
            antenna.SNR = Math.Max(0, receivedPower - noiseFloor);
            antenna.RSSI = -110.0 + signalStrength * 0.6;

            return Math.Max(0, Math.Min(100, signalStrength));
        }
        /// <summary>
        /// Initial 360° × 90° scan to locate aircraft
        /// </summary>
        private void PerformInitialScan(AntennaState antenna, AirplaneState airplane,
                                      double antennaLat, double antennaLon, double antennaAlt)
        {
            if ((DateTime.Now - lastPsoUpdate).TotalSeconds >= PSO_UPDATE_RATE)
            {
                lastPsoUpdate = DateTime.Now;

                // PSO fitness fonksiyonu - koordinat bazlı optimizasyon
                Func<double, double, double> initialScanFitness = (h, v) =>
                {
                    antenna.HorizontalAngle = h;
                    antenna.VerticalAngle = v;

                    // Sinyal gücünü ölç
                    double signal = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);
                    signal = signalFilter.Update(signal);

                    // Tarama alanını sinyal gücüne göre ayarla
                    if (signal > bestSignalStrength)
                    {
                        bestSignalStrength = signal;
                        bestHorizontalAngle = h;
                        bestVerticalAngle = v;

                        // Kademeli daralma
                        horizontalScanArea = Math.Max(MIN_SCAN_AREA_H,
                            horizontalScanArea * (1.0 - (signal / 100.0) * 0.2)); // Her adımda max %20 daralma
                        verticalScanArea = Math.Max(MIN_SCAN_AREA_V,
                            verticalScanArea * (1.0 - (signal / 100.0) * 0.2));
                    }

                    return signal;
                };

                var (optH, optV) = pso.ScanStep(
                    initialScanFitness,
                    horizontalScanArea,
                    verticalScanArea,
                    bestHorizontalAngle > 0 ? bestHorizontalAngle : 180.0, // İlk taramada ortadan başla
                    bestVerticalAngle > 0 ? bestVerticalAngle : 45.0,
                    bestSignalStrength > FAIR_SIGNAL
                );

                // Kalman filtresi ile yumuşatılmış hareket
                antenna.HorizontalAngle = horizontalFilter.Update(optH);
                antenna.VerticalAngle = verticalFilter.Update(optV);
                antenna.SignalStrength = bestSignalStrength;

                // Yeterli sinyal bulunduğunda initial scan'i bitir
                if (bestSignalStrength > GOOD_SIGNAL ||
                    (horizontalScanArea <= MIN_SCAN_AREA_H * 2 && bestSignalStrength > FAIR_SIGNAL))
                {
                    isInitialScan = false;
                    currentScanMode = "Signal Acquired";
                }
            }
        }

        /// <summary>
        /// Generates a uniform spiral grid covering 360x90 degrees for the initial scan.
        /// </summary>
        private void GenerateInitialScanGrid()
        {
            initialScanGrid.Clear();
            // Fibonacci sphere for even distribution
            int n = INITIAL_SCAN_POINTS;
            double goldenAngle = Math.PI * (3 - Math.Sqrt(5));
            for (int i = 0; i < n; i++)
            {
                double y = 1 - (i / (double)(n - 1)) * 2; // y from 1 to -1
                double radius = Math.Sqrt(1 - y * y);
                double theta = goldenAngle * i;
                double x = Math.Cos(theta) * radius;
                double z = Math.Sin(theta) * radius;

                // Convert to spherical coordinates
                double h = (Math.Atan2(z, x) * 180.0 / Math.PI + 360.0) % 360.0;
                double v = Math.Acos(y) * 180.0 / Math.PI; // 0 (up) to 180 (down)
                v = Math.Min(90.0, v); // Clamp to 0-90 for hemisphere

                initialScanGrid.Add((h, v));
            }
        }

        /// <summary>
        /// Updates directional antenna to track targets found by scanning antenna
        /// </summary>
        public void UpdateDirectionalAntenna(AntennaState directionalAntenna, AntennaState scanningAntenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            // Direkt olarak en iyi konumu kullan
            double targetH = bestHorizontalAngle;
            double targetV = bestVerticalAngle;

            // Kalman filtresi ile yumuşat
            double smoothedH = horizontalFilter.Update(targetH);
            double smoothedV = verticalFilter.Update(targetV);

            // Anteni yönlendir
            directionalAntenna.HorizontalAngle = smoothedH;
            directionalAntenna.VerticalAngle = smoothedV;

            // Sinyal gücünü hesapla
            double signal = SimulateSignalStrength(
                directionalAntenna, airplane,
                directionalAntenna.Latitude,
                directionalAntenna.Longitude,
                directionalAntenna.Altitude
            );

            // Metrikleri güncelle
            directionalAntenna.SignalStrength = signal;
            UpdateSignalMetrics(directionalAntenna);

            Console.WriteLine($"Directional antenna at H:{smoothedH:F1}° V:{smoothedV:F1}° Signal:{signal:F1}");
        }

        private double CalculateDistance(double lat1, double lon1, double alt1,
                                       double lat2, double lon2, double alt2)
        {
            double metersPerLat = 111320.0;
            double metersPerLon = 111320.0 * Math.Cos(lat1 * Math.PI / 180.0);

            double dx = (lon2 - lon1) * metersPerLon;
            double dy = (lat2 - lat1) * metersPerLat;
            double dz = alt2 - alt1;

            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        /// Store position history to calculate aircraft velocity and predict movement
        /// </summary>
        private void UpdatePositionHistory(double targetH, double targetV)
        {
            // Şu anki pozisyonu zaman damgasıyla sakla
            positionHistory.Enqueue((targetH, targetV, DateTime.Now));

            // Geçmiş boyutunu sınırla - daha kısa tarihçe daha hızlı adaptasyon
            if (positionHistory.Count > 5) // Daha az geçmiş noktası (önceki: MAX_HISTORY_POINTS)
            {
                positionHistory.Dequeue();
            }
        }

        /// <summary>
        /// Calculate aircraft velocity and predict future position
        /// </summary>
        private void PredictTargetMovement()
        {
            // Hız hesaplamak için en az 2 geçmiş noktası gerekli
            if (positionHistory.Count < 2) return;

            var positions = positionHistory.ToArray();
            var oldest = positions[0];
            var newest = positions[positions.Length - 1];

            // En eski ve en yeni konum arasındaki zaman farkını hesapla
            double timeDiff = (newest.time - oldest.time).TotalSeconds;
            if (timeDiff < 0.1) return; // Küçük sayılara bölünmeyi önle

            // Sarmalama işlemini ele alarak yatay açı değişimini hesapla (0-360 derece)
            double hDiff = newest.h - oldest.h;
            if (hDiff > 180) hDiff -= 360;
            if (hDiff < -180) hDiff += 360;

            // Dikey açı değişimini hesapla (0-90 derece)
            double vDiff = newest.v - oldest.v;

            // Açısal hızları hesapla - daha duyarlı değerler
            horizontalVelocity = hDiff / timeDiff;
            verticalVelocity = vDiff / timeDiff;

            // Düzeltme: Daha kısa süre (0.2 saniye) için gelecek pozisyonu tahmin et
            double predictionTime = 0.2; // Önceki: 0.5
            predictedHorizontalAngle = (newest.h + horizontalVelocity * predictionTime + 360) % 360;
            predictedVerticalAngle = Math.Max(0, Math.Min(90, newest.v + verticalVelocity * predictionTime));

            lastPositionUpdate = DateTime.Now;
        }

        /// <summary>
        /// Wide area search when signal is weak or lost
        /// </summary>
        private void PerformWideAreaSearch(AntennaState antenna, AirplaneState airplane,
                                         double antennaLat, double antennaLon, double antennaAlt,
                                         double targetH, double targetV)
        {
            // Set scan areas wide enough to reacquire signal
            horizontalScanArea = Math.Min(MAX_SCAN_AREA_H, 120.0);
            verticalScanArea = Math.Min(MAX_SCAN_AREA_V, 60.0);

            // Use PSO but only update periodically to prevent too frequent changes
            if ((DateTime.Now - lastPsoUpdate).TotalSeconds >= PSO_UPDATE_RATE)
            {
                lastPsoUpdate = DateTime.Now;

                // Use calculated target position as center for search
                double searchCenterH = targetH;
                double searchCenterV = targetV;

                // Custom wide-area fitness function emphasizing exploration
                Func<double, double, double> wideFitnessFunction = (h, v) =>
                {
                    // Move antenna to evaluate position
                    antenna.HorizontalAngle = h;
                    antenna.VerticalAngle = v;

                    // Get signal strength at this position
                    double signalStrength = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);

                    // Apply Kalman filtering
                    signalStrength = signalFilter.Update(signalStrength);

                    // Update best signal if improved
                    if (signalStrength > bestSignalStrength)
                    {
                        bestSignalStrength = signalStrength;
                        bestHorizontalAngle = h;
                        bestVerticalAngle = v;
                    }

                    return signalStrength;
                };

                // Run PSO optimization step
                var (optH, optV) = pso.ScanStep(
                    wideFitnessFunction,
                    horizontalScanArea,
                    verticalScanArea,
                    searchCenterH,
                    searchCenterV,
                    false,  // Don't use velocity prediction in wide search
                    -1, -1
                );

                // Move antenna to optimal position found by PSO
                antenna.HorizontalAngle = optH;
                antenna.VerticalAngle = optV;
            }
            else
            {
                // Between PSO updates, use a spiral search pattern
                double time = (DateTime.Now - lastPsoUpdate).TotalSeconds / PSO_UPDATE_RATE;
                double spiralRadius = Math.Min(30.0, time * 10.0);
                double spiralAngle = time * 720.0; // 2 full rotations per step

                // Calculate spiral pattern around last known position
                double offsetH = spiralRadius * Math.Cos(spiralAngle * Math.PI / 180.0);
                double offsetV = spiralRadius * Math.Sin(spiralAngle * Math.PI / 180.0) * 0.5; // Flatten vertically

                // Set antenna to spiral search position
                antenna.HorizontalAngle = (targetH + offsetH + 360.0) % 360.0;
                antenna.VerticalAngle = Math.Max(0, Math.Min(90.0, targetV + offsetV));
            }
        }

        /// <summary>
        /// Signal acquisition when signal is moderate
        /// </summary>
        private void PerformSignalAcquisition(AntennaState antenna, AirplaneState airplane,
                                           double antennaLat, double antennaLon, double antennaAlt,
                                           double targetH, double targetV)
        {
            // Düzeltme 1: Her durumda güncel hedef açılarını al
            GetTargetAngles(antennaLat, antennaLon, antennaAlt, airplane, out double realTargetH, out double realTargetV);
            lastKnownTargetH = realTargetH;
            lastKnownTargetV = realTargetV;

            // Düzeltme 2: Gerçek sinyal gücünü ölç
            double currentSignal = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);

            // Düzeltme 3: Sinyal hızlı bir şekilde düştüyse güncelleme yap
            if (currentSignal < bestSignalStrength * 0.7)
            {
                bestSignalStrength = currentSignal;
                // Tarama alanını genişlet
                horizontalScanArea = Math.Min(MAX_SCAN_AREA_H, 90.0);
                verticalScanArea = Math.Min(MAX_SCAN_AREA_V, 45.0);
            }

            // Düzeltme 4: Çok uzun süre bu modda kalındıysa reset at
            if ((DateTime.Now - lastPsoUpdate).TotalSeconds > 3.0)
            {
                // Uzun süre sinyal bulunamadı, hedef konumuna yakın geniş tarama yap
                horizontalScanArea = Math.Min(MAX_SCAN_AREA_H, 180.0);
                verticalScanArea = Math.Min(MAX_SCAN_AREA_V, 90.0);

                // PSO'yu kısmen sıfırla
                pso.PartialReset();
                bestHorizontalAngle = lastKnownTargetH;
                bestVerticalAngle = lastKnownTargetV;
                lastPsoUpdate = DateTime.Now;
            }

            // Düzeltme 5: Tarama alanını dinamik olarak ayarla - minimum değerler arttırıldı
            horizontalScanArea = Math.Max(MIN_SCAN_AREA_H * 3, 60.0 * (1.0 - bestSignalStrength / 100.0));
            verticalScanArea = Math.Max(MIN_SCAN_AREA_V * 3, 30.0 * (1.0 - bestSignalStrength / 100.0));

            // Düzeltme 6: PSO güncelleme hızını arttır
            if ((DateTime.Now - lastPsoUpdate).TotalSeconds >= PSO_UPDATE_RATE * 0.5)
            {
                lastPsoUpdate = DateTime.Now;

                // Düzeltme 7: Arama merkezini gerçek hedef açılarına daha yakın yap
                double searchCenterH = WeightedAngleAverage(
                    realTargetH,  // Gerçek hedef açısı - daha önce lastKnownTargetH kullanılıyordu
                    bestHorizontalAngle,
                    0.6,          // Gerçek hedefe daha fazla ağırlık (öncekinden yüksek)
                    0.4);         // En iyi sinyale az ağırlık

                double searchCenterV =
                    realTargetV * 0.6 + // Gerçek hedefe daha fazla ağırlık
                    bestVerticalAngle * 0.4; // En iyi sinyale az ağırlık

                // Düzeltme 8: Fitness fonksiyonunu geliştir
                Func<double, double, double> acquisitionFitnessFunction = (h, v) =>
                {
                    // Anteni değerlendirme konumuna getir
                    antenna.HorizontalAngle = h;
                    antenna.VerticalAngle = v;

                    // Bu konumdaki sinyal gücünü al
                    double signalStrength = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);

                    // Kalman filtrelemeyi azalt - daha hızlı tepki
                    signalStrength = signalFilter.Update(signalStrength * 0.7 + signalStrength * 0.3);

                    // Düzeltme 9: En iyi sinyali daha agresif güncelle
                    if (signalStrength > bestSignalStrength * 0.8)
                    {
                        bestSignalStrength = signalStrength * 0.7 + bestSignalStrength * 0.3; // Yeni sinyale daha fazla ağırlık
                        bestHorizontalAngle = h;
                        bestVerticalAngle = v;

                        // Düzeltme 10: İyi sinyal bulunduğunda modlar arası geçişi kolaylaştır
                        if (signalStrength > TRACKING_THRESHOLD * 0.9)
                        {
                            // Tracking moduna geçmeye hazırız
                            targetLocked = true;
                            currentScanMode = "Target Tracking";
                        }
                    }

                    return signalStrength;
                };

                // Düzeltme 11: PSO'yu daha geniş bir aramayla çalıştır
                var (optH, optV) = pso.ScanStep(
                    acquisitionFitnessFunction,
                    Math.Max(horizontalScanArea, 30.0), // Minimum 30 derece arama alanı
                    Math.Max(verticalScanArea, 15.0),   // Minimum 15 derece arama alanı
                    searchCenterH,
                    searchCenterV,
                    bestSignalStrength > FAIR_SIGNAL, // İyi sinyalde hareket tahmini kullan
                    predictedHorizontalAngle > 0 ? predictedHorizontalAngle : searchCenterH,
                    predictedVerticalAngle > 0 ? predictedVerticalAngle : searchCenterV
                );

                // Düzeltme 12: Kalman filtreleme parametrelerini değiştir
                double filteredH = horizontalFilter.Update(optH * 0.8 + optH * 0.2); // Daha az filtreleme 
                double filteredV = verticalFilter.Update(optV * 0.8 + optV * 0.2);

                // Anteni filtrelenmiş optimum konuma getir
                antenna.HorizontalAngle = filteredH;
                antenna.VerticalAngle = filteredV;

                // Düzeltme 13: İyi bir sinyal bulunduysa tracking moduna geçmeyi zorla
                if (currentSignal > TRACKING_THRESHOLD * 0.8)
                {
                    targetLocked = true;
                    currentScanMode = "Target Tracking";
                }
            }
            // Düzeltme 14: Acquisition modunda uzun süre kalındıysa arama algoritmasına destek
            else
            {
                // PSO güncellemesi bekliyorken spiral arama yap
                double spiralRadius = 5.0;
                double spiralPhase = (DateTime.Now - lastPsoUpdate).TotalSeconds * 720.0;
                double offsetH = spiralRadius * Math.Cos(spiralPhase * Math.PI / 180.0);
                double offsetV = spiralRadius * Math.Sin(spiralPhase * Math.PI / 180.0) * 0.5;

                antenna.HorizontalAngle = (antenna.HorizontalAngle + offsetH + 360.0) % 360.0;
                antenna.VerticalAngle = Math.Max(0, Math.Min(90.0, antenna.VerticalAngle + offsetV));
            }
        }
        /// <summary>
        /// Precision tracking when signal is strong
        /// </summary>
        private void PerformTargetTracking(AntennaState antenna, AirplaneState airplane,
                                        double antennaLat, double antennaLon, double antennaAlt,
                                        double targetH, double targetV)
        {
            // Sinyal kararlılığını kontrol et
            if (recentSignals.Count >= STABLE_SIGNAL_COUNT)
            {
                recentSignals.Dequeue();
            }
            recentSignals.Enqueue(bestSignalStrength);

            bool isSignalStable = recentSignals.Count == STABLE_SIGNAL_COUNT &&
                                 recentSignals.Max() - recentSignals.Min() < SIGNAL_STABILITY_THRESHOLD;

            // Gerçek sinyal gücünü ölç
            double realSignalStrength = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);

            // Ani sinyal düşüşü kontrolü
            if (realSignalStrength < bestSignalStrength * 0.5 && bestSignalStrength > TRACKING_THRESHOLD)
            {
                GetTargetAngles(antennaLat, antennaLon, antennaAlt, airplane, out targetH, out targetV);

                // Açı değişimini sınırla
                double hDiff = GetAngleDifference(targetH, antenna.HorizontalAngle);
                double vDiff = targetV - antenna.VerticalAngle;

                if (Math.Abs(hDiff) > MAX_ANGLE_CHANGE || Math.Abs(vDiff) > MAX_ANGLE_CHANGE)
                {
                    // Değişimi sınırla
                    targetH = antenna.HorizontalAngle + Math.Sign(hDiff) * MAX_ANGLE_CHANGE;
                    targetV = antenna.VerticalAngle + Math.Sign(vDiff) * MAX_ANGLE_CHANGE;
                }

                lastKnownTargetH = targetH;
                lastKnownTargetV = targetV;

                if (!hasValidPosition)
                {
                    lastValidHorizontalAngle = antenna.HorizontalAngle;
                    lastValidVerticalAngle = antenna.VerticalAngle;
                    hasValidPosition = true;
                }
            }

            // PSO güncellemesi için zaman kontrolü
            if ((DateTime.Now - lastPsoUpdate).TotalSeconds >= PSO_UPDATE_RATE)
            {
                lastPsoUpdate = DateTime.Now;

                // Kararlı sinyalde ani değişimleri engelle
                if (isSignalStable && hasValidPosition)
                {
                    double maxChange = MAX_ANGLE_CHANGE * 0.5; // Kararlı durumda daha yavaş hareket
                    targetH = ClampAngleChange(lastValidHorizontalAngle, targetH, maxChange);
                    targetV = ClampAngleChange(lastValidVerticalAngle, targetV, maxChange);
                }

                // Fitness fonksiyonunu güncelle
                Func<double, double, double> trackingFitnessFunc = (h, v) =>
                {
                    // Mevcut konumdan çok uzaklaşmayı cezalandır
                    double anglePenalty = 0;
                    if (hasValidPosition)
                    {
                        double hDiff = Math.Abs(GetAngleDifference(h, lastValidHorizontalAngle));
                        double vDiff = Math.Abs(v - lastValidVerticalAngle);
                        anglePenalty = (hDiff + vDiff) * 0.1; // Uzaklaştıkça artan ceza
                    }

                    antenna.HorizontalAngle = h;
                    antenna.VerticalAngle = v;

                    double signal = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);
                    signal = signalFilter.Update(signal);

                    // Ceza puanını uygula
                    return Math.Max(0, signal - anglePenalty);
                };

                // PSO güncellemesi
                var (optH, optV) = pso.ScanStep(
                    trackingFitnessFunc,
                    horizontalScanArea,
                    verticalScanArea,
                    targetH,
                    targetV,
                    true,
                    predictedHorizontalAngle,
                    predictedVerticalAngle
                );

                // Yeni konum geçerliyse kaydet
                if (realSignalStrength > TRACKING_THRESHOLD * 0.8)
                {
                    lastValidHorizontalAngle = antenna.HorizontalAngle;
                    lastValidVerticalAngle = antenna.VerticalAngle;
                    hasValidPosition = true;
                }

                // Kalman filtreleme
                double filteredH = horizontalFilter.Update(optH);
                double filteredV = verticalFilter.Update(optV);

                // Anteni filtrelenmiş optimum konuma getir
                antenna.HorizontalAngle = filteredH;
                antenna.VerticalAngle = filteredV;

                // Pozisyon geçmişi ve tahmin mekanizmasını güncelle
                UpdatePositionHistory(bestHorizontalAngle, bestVerticalAngle);
                PredictTargetMovement();
            }
        }

        // Açı değişimini sınırlayan yardımcı metod
        private double ClampAngleChange(double current, double target, double maxChange)
        {
            double diff = GetAngleDifference(target, current);
            if (Math.Abs(diff) > maxChange)
            {
                return (current + Math.Sign(diff) * maxChange + 360.0) % 360.0;
            }
            return target;
        }

        // İki açı arasındaki en kısa farkı hesaplayan yardımcı metod
        private double GetAngleDifference(double angle1, double angle2)
        {
            double diff = ((angle1 - angle2 + 540.0) % 360.0) - 180.0;
            return diff;
        }

        /// <summary>
        /// Calculate antenna horizontal and vertical angles needed to point at aircraft
        /// </summary>
        private void GetTargetAngles(double antennaLat, double antennaLon, double antennaAlt,
                                   AirplaneState airplane, out double horizontalAngle, out double verticalAngle)
        {
            // Coğrafi koordinatları metre cinsi mesafeye çevir
            double metersPerLat = 111320.0;
            double metersPerLon = 111320.0 * Math.Cos(antennaLat * Math.PI / 180.0);

            // Mesafeyi hesapla
            double dx = (airplane.Longitude - antennaLon) * metersPerLon;
            double dy = (airplane.Latitude - antennaLat) * metersPerLat;
            double dz = airplane.Altitude - antennaAlt;

            // Yatay açıyı hesapla (kuzeyden saat yönünde)
            horizontalAngle = (Math.Atan2(dx, dy) * 180.0 / Math.PI + 360.0) % 360.0;

            // Dikey açıyı hesapla (yataydan yukarı)
            double groundDistance = Math.Sqrt(dx * dx + dy * dy);
            verticalAngle = Math.Atan2(dz, groundDistance) * 180.0 / Math.PI;
            verticalAngle = Math.Max(0, Math.Min(90.0, verticalAngle));
        }

        /// <summary>
        /// Update SNR and RSSI metrics based on signal strength
        /// </summary>
        private void UpdateSignalMetrics(AntennaState antenna)
        {
            // Signal quality is affected by recent history (exponential smoothing)
            signalQualityHistory = bestSignalStrength * (1.0 - SIGNAL_HISTORY_WEIGHT) +
                                  signalQualityHistory * SIGNAL_HISTORY_WEIGHT;

            // Calculate RSSI (Received Signal Strength Indicator) in dBm
            // Typical range: -110 dBm (very weak) to -50 dBm (strong)
            double rssi = -110.0 + signalQualityHistory * 0.6;

            // Calculate SNR (Signal-to-Noise Ratio) in dB
            // Typical range: 0 dB (unusable) to 25+ dB (excellent)
            double snr = Math.Max(0, signalQualityHistory * 0.25);

            // Update antenna signal metrics
            antenna.RSSI = rssi;
            antenna.SNR = snr;
        }

        /// <summary>
        /// Calculates weighted average of two angles, handling 0-360 wraparound
        /// </summary>
        private double WeightedAngleAverage(double angle1, double angle2, double weight1, double weight2)
        {
            // Normalize weights
            double totalWeight = weight1 + weight2;
            weight1 /= totalWeight;
            weight2 /= totalWeight;

            // Convert to cartesian coordinates to handle wraparound
            double x1 = Math.Cos(angle1 * Math.PI / 180.0);
            double y1 = Math.Sin(angle1 * Math.PI / 180.0);
            double x2 = Math.Cos(angle2 * Math.PI / 180.0);
            double y2 = Math.Sin(angle2 * Math.PI / 180.0);

            // Weighted average in cartesian space
            double x = x1 * weight1 + x2 * weight2;
            double y = y1 * weight1 + y2 * weight2;

            // Convert back to angle
            double resultAngle = (Math.Atan2(y, x) * 180.0 / Math.PI + 360.0) % 360.0;
            return resultAngle;
        }

        /// <summary>
        /// Reset controller to initial state
        /// </summary>
        public void Reset()
        {
            // Reset filters
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();

            // Reset PSO
            pso.Reset();

            // Reset scan state
            isInitialScan = true;
            scanStartTime = default;
            bestSignalStrength = 0;
            signalQualityHistory = 0;

            // Reset angles
            bestHorizontalAngle = 0;
            bestVerticalAngle = 0;
            predictedHorizontalAngle = 0;
            predictedVerticalAngle = 0;
            lastKnownTargetH = 0;
            lastKnownTargetV = 0;

            // Reset velocities
            horizontalVelocity = 0;
            verticalVelocity = 0;

            // Reset scan areas
            horizontalScanArea = INITIAL_SCAN_AREA_H;
            verticalScanArea = INITIAL_SCAN_AREA_V;

            // Reset histories
            positionHistory.Clear();

            // Reset tracking state
            targetLocked = false;
            currentScanMode = "Initial Scan";
            scanPhaseDegrees = 0;

            // Reset update timers
            lastScanUpdate = DateTime.MinValue;
            lastPsoUpdate = DateTime.MinValue;
            lastPositionUpdate = DateTime.MinValue;

            // Reset exploration phase
            isExplorationPhase = true;
            explorationElapsed = 0;
            explorationAngle = 0;

            // Reset signal stability tracking
            recentSignals.Clear();
            hasValidPosition = false;
            lastValidHorizontalAngle = 0;
            lastValidVerticalAngle = 0;
        }

        /// <summary>
        /// Returns debugging info about current controller state
        /// </summary>
        public string GetDebugInfo()
        {
            return $"Mode: {currentScanMode}\n" +
                   $"Signal: {bestSignalStrength:F1}%\n" +
                   $"Target: H={lastKnownTargetH:F1}° V={lastKnownTargetV:F1}°\n" +
                   $"Current: H={bestHorizontalAngle:F1}° V={bestVerticalAngle:F1}°\n" +
                   $"Predicted: H={predictedHorizontalAngle:F1}° V={predictedVerticalAngle:F1}°\n" +
                   $"Velocity: H={horizontalVelocity:F2}°/s V={verticalVelocity:F2}°/s\n" +
                   $"Scan Area: H={horizontalScanArea:F1}° V={verticalScanArea:F1}°\n" +
                   $"PSO Iteration: {PsoIteration}\n" +
                   $"Convergence: {ConvergenceRate:P2}\n" +
                   $"Locked: {targetLocked}";
        }

        /// <summary>
        /// Helper method to calculate scan area based on signal strength
        /// </summary>
        private double CalculateScanArea(double signalStrength)
        {
            if (signalStrength > 80) return Math.Max(MIN_SCAN_AREA_H, 20.0);
            if (signalStrength > 60) return Math.Max(MIN_SCAN_AREA_H, 45.0);
            if (signalStrength > 40) return Math.Max(MIN_SCAN_AREA_H, 90.0);
            return Math.Max(MIN_SCAN_AREA_H, 180.0);
        }
    }
}