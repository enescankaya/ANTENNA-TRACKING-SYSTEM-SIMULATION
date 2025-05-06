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
        private readonly ParticleSwarmOptimizer pso;

        // Scan area parameters - değişkenleri güncelle
        private const double INITIAL_SCAN_AREA_H = 360.0;
        private const double INITIAL_SCAN_AREA_V = 90.0;
        private const double MIN_SCAN_AREA_H = 10.0;
        private const double MIN_SCAN_AREA_V = 5.0;
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
        private const double SIGNAL_LOSS_THRESHOLD = 20.0; // Eşiği yükselt
        private const double TRACKING_THRESHOLD = 50.0;    // Takip eşiğini yükselt

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
        private const double PSO_UPDATE_RATE = 0.1;    // 10Hz'e düşür

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
            // Kalman filtre parametrelerini düzelt
            horizontalFilter = new KalmanFilter(0.05, 0.5, 0.2);  // Daha yavaş tepki
            verticalFilter = new KalmanFilter(0.05, 0.5, 0.2);    // Daha yavaş tepki
            signalFilter = new KalmanFilter(0.1, 0.3, 0.1);       // Sinyal filtresini hassaslaştır

            pso = new ParticleSwarmOptimizer(100); // Parçacık sayısını artır

            Reset();
        }

        /// <summary>
        /// Updates the scanning antenna's position to search for and track aircraft
        /// </summary>
        public void UpdateScanningAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            // Rate limit updates to maintain system performance
            if ((DateTime.Now - lastScanUpdate).TotalSeconds < SCAN_UPDATE_RATE) return;
            lastScanUpdate = DateTime.Now;

            double antennaLat = antenna.Latitude;
            double antennaLon = antenna.Longitude;
            double antennaAlt = antenna.Altitude;

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
                return;
            }

            // 2. PSO ile kademeli daralan tarama (mevcut mantık)
            if (isInitialScan)
            {
                currentScanMode = "Initial 360° Scan";
                // Initial scan phase - systematic grid search of entire space
                PerformInitialScan(antenna, airplane, antennaLat, antennaLon, antennaAlt);
            }
            else if (bestSignalStrength < SIGNAL_LOSS_THRESHOLD)
            {
                currentScanMode = "Wide Area Search";
                targetLocked = false;
                // Poor signal - use wider search pattern around last known position
                PerformWideAreaSearch(antenna, airplane, antennaLat, antennaLon, antennaAlt, lastKnownTargetH, lastKnownTargetV);
            }
            else if (bestSignalStrength < TRACKING_THRESHOLD)
            {
                currentScanMode = "Signal Acquisition";
                targetLocked = false;
                // Medium signal - PSO with moderate search area
                PerformSignalAcquisition(antenna, airplane, antennaLat, antennaLon, antennaAlt, lastKnownTargetH, lastKnownTargetV);
            }
            else
            {
                currentScanMode = "Target Tracking";
                targetLocked = true;
                // Good signal - precision tracking with PSO
                PerformTargetTracking(antenna, airplane, antennaLat, antennaLon, antennaAlt, lastKnownTargetH, lastKnownTargetV);
            }

            // Update signal metrics (RSSI, SNR) for display
            UpdateSignalMetrics(antenna);

            // Update scan phase display (0-360)
            scanPhaseDegrees = (int)(antenna.HorizontalAngle % 360);
        }

        // SimulateSignalStrength fonksiyonunu gerçekçi hale getir
        private double SimulateSignalStrength(AntennaState antenna, AirplaneState airplane,
                                           double antennaLat, double antennaLon, double antennaAlt)
        {
            GetTargetAngles(antennaLat, antennaLon, antennaAlt, airplane, out double targetH, out double targetV);

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
            // Store current position with timestamp
            positionHistory.Enqueue((targetH, targetV, DateTime.Now));

            // Limit history size
            if (positionHistory.Count > MAX_HISTORY_POINTS)
            {
                positionHistory.Dequeue();
            }
        }

        /// <summary>
        /// Calculate aircraft velocity and predict future position
        /// </summary>
        private void PredictTargetMovement()
        {
            // Need at least 2 history points to calculate velocity
            if (positionHistory.Count < 2) return;

            var positions = positionHistory.ToArray();
            var oldest = positions[0];
            var newest = positions[positions.Length - 1];

            // Calculate time difference between oldest and newest position
            double timeDiff = (newest.time - oldest.time).TotalSeconds;
            if (timeDiff < 0.1) return; // Avoid division by small numbers

            // Calculate horizontal angle change with wraparound handling (0-360 degrees)
            double hDiff = newest.h - oldest.h;
            if (hDiff > 180) hDiff -= 360;
            if (hDiff < -180) hDiff += 360;

            // Calculate vertical angle change (0-90 degrees)
            double vDiff = newest.v - oldest.v;

            // Calculate angular velocities
            horizontalVelocity = hDiff / timeDiff;
            verticalVelocity = vDiff / timeDiff;

            // Predict position 0.5 seconds into future
            double predictionTime = 0.5;
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
            // Adjust scan area based on signal strength - better signal means smaller search area
            horizontalScanArea = Math.Max(MIN_SCAN_AREA_H, 60.0 * (1.0 - bestSignalStrength / 100.0));
            verticalScanArea = Math.Max(MIN_SCAN_AREA_V, 30.0 * (1.0 - bestSignalStrength / 100.0));

            // Use PSO for signal acquisition
            if ((DateTime.Now - lastPsoUpdate).TotalSeconds >= PSO_UPDATE_RATE)
            {
                lastPsoUpdate = DateTime.Now;

                // Calculate search center as weighted average of computed target and best signal
                double searchCenterH = WeightedAngleAverage(
                    targetH, bestHorizontalAngle, TARGET_WEIGHT, SIGNAL_WEIGHT);
                double searchCenterV = targetV * TARGET_WEIGHT + bestVerticalAngle * SIGNAL_WEIGHT;

                // Custom fitness function for acquisition phase
                Func<double, double, double> acquisitionFitnessFunction = (h, v) =>
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

                // Run PSO optimization step with limited velocity prediction
                var (optH, optV) = pso.ScanStep(
                    acquisitionFitnessFunction,
                    horizontalScanArea,
                    verticalScanArea,
                    searchCenterH,
                    searchCenterV,
                    bestSignalStrength > 25, // Use velocity prediction if signal is decent
                    predictedHorizontalAngle,
                    predictedVerticalAngle
                );

                // Move antenna to optimal position found by PSO
                antenna.HorizontalAngle = optH;
                antenna.VerticalAngle = optV;
            }
        }

        /// <summary>
        /// Precision tracking when signal is strong
        /// </summary>
        private void PerformTargetTracking(AntennaState antenna, AirplaneState airplane,
                                        double antennaLat, double antennaLon, double antennaAlt,
                                        double targetH, double targetV)
        {
            // Narrow scan area for precision tracking
            horizontalScanArea = Math.Max(MIN_SCAN_AREA_H, 30.0 * (1.0 - bestSignalStrength / 100.0));
            verticalScanArea = Math.Max(MIN_SCAN_AREA_V, 15.0 * (1.0 - bestSignalStrength / 100.0));

            // Use PSO for precision tracking with higher update rate
            if ((DateTime.Now - lastPsoUpdate).TotalSeconds >= PSO_UPDATE_RATE * 0.5) // Faster updates for tracking
            {
                lastPsoUpdate = DateTime.Now;

                // Calculate search center incorporating prediction for moving targets
                double searchCenterH = WeightedAngleAverage(
                    targetH,
                    WeightedAngleAverage(bestHorizontalAngle, predictedHorizontalAngle, 0.7, 0.3),
                    TARGET_WEIGHT,
                    SIGNAL_WEIGHT + PREDICTION_WEIGHT);

                double searchCenterV =
                    targetV * TARGET_WEIGHT +
                    (bestVerticalAngle * 0.7 + predictedVerticalAngle * 0.3) * (SIGNAL_WEIGHT + PREDICTION_WEIGHT);

                // Specialized fitness function for precision tracking
                Func<double, double, double> trackingFitnessFunction = (h, v) =>
                {
                    // Move antenna to evaluate position
                    antenna.HorizontalAngle = h;
                    antenna.VerticalAngle = v;

                    // Get signal strength at this position
                    double signalStrength = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);

                    // Apply Kalman filtering with lower noise parameters for stable tracking
                    signalStrength = signalFilter.Update(signalStrength);

                    // Update best signal with greater emphasis on new readings (less history weight)
                    if (signalStrength > bestSignalStrength * 0.95) // Allow slight decrease to prevent getting stuck
                    {
                        bestSignalStrength = signalStrength * 0.8 + bestSignalStrength * 0.2; // Smoothed update
                        bestHorizontalAngle = h;
                        bestVerticalAngle = v;
                    }

                    return signalStrength;
                };

                // Run PSO optimization step with movement prediction
                var (optH, optV) = pso.ScanStep(
                    trackingFitnessFunction,
                    horizontalScanArea,
                    verticalScanArea,
                    searchCenterH,
                    searchCenterV,
                    true, // Always use velocity prediction in tracking mode
                    predictedHorizontalAngle,
                    predictedVerticalAngle
                );

                // Apply Kalman filtering to smooth antenna position updates
                double filteredH = horizontalFilter.Update(optH);
                double filteredV = verticalFilter.Update(optV);

                // Move antenna to filtered optimal position
                antenna.HorizontalAngle = filteredH;
                antenna.VerticalAngle = filteredV;
            }
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