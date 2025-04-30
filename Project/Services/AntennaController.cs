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

        // Scan area parameters
        private const double MIN_SCAN_AREA_H = 10.0;  // Minimum horizontal scan area in degrees
        private const double MIN_SCAN_AREA_V = 5.0;   // Minimum vertical scan area in degrees
        private const double MAX_SCAN_AREA_H = 360.0; // Maximum horizontal scan area in degrees
        private const double MAX_SCAN_AREA_V = 90.0;  // Maximum vertical scan area in degrees

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
        private double horizontalScanArea = MAX_SCAN_AREA_H;
        private double verticalScanArea = MAX_SCAN_AREA_V;
        private int scanPhaseDegrees = 0;
        private bool targetLocked = false;
        private string currentScanMode = "Initial Scan";

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

            // Calculate where the aircraft should be relative to antenna position
            GetTargetAngles(antennaLat, antennaLon, antennaAlt, airplane, out double targetH, out double targetV);

            // Store these target angles for later use
            lastKnownTargetH = targetH;
            lastKnownTargetV = targetV;

            // Update position history for movement prediction
            UpdatePositionHistory(targetH, targetV);

            // Predict aircraft movement for proactive tracking
            PredictTargetMovement();

            // Choose tracking strategy based on scan phase
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
                PerformWideAreaSearch(antenna, airplane, antennaLat, antennaLon, antennaAlt, targetH, targetV);
            }
            else if (bestSignalStrength < TRACKING_THRESHOLD)
            {
                currentScanMode = "Signal Acquisition";
                targetLocked = false;
                // Medium signal - PSO with moderate search area
                PerformSignalAcquisition(antenna, airplane, antennaLat, antennaLon, antennaAlt, targetH, targetV);
            }
            else
            {
                currentScanMode = "Target Tracking";
                targetLocked = true;
                // Good signal - precision tracking with PSO
                PerformTargetTracking(antenna, airplane, antennaLat, antennaLon, antennaAlt, targetH, targetV);
            }

            // Update signal metrics (RSSI, SNR) for display
            UpdateSignalMetrics(antenna);

            // Update scan phase display (0-360)
            scanPhaseDegrees = (int)(antenna.HorizontalAngle % 360);
        }

        /// <summary>
        /// Initial 360° × 90° scan to locate aircraft
        /// </summary>
        private void PerformInitialScan(AntennaState antenna, AirplaneState airplane,
                                      double antennaLat, double antennaLon, double antennaAlt)
        {
            if (scanStartTime == default)
            {
                scanStartTime = DateTime.Now;
                currentGridStepH = 0;
                currentGridStepV = 0;
                bestSignalStrength = 0;
                bestHorizontalAngle = 0;  // Sıfırla
                bestVerticalAngle = 0;    // Sıfırla
                horizontalScanArea = MAX_SCAN_AREA_H;
                verticalScanArea = MAX_SCAN_AREA_V;
            }

            double elapsed = (DateTime.Now - scanStartTime).TotalSeconds;
            double totalPoints = GRID_STEPS_H * GRID_STEPS_V;
            double scanProgress = Math.Min(1.0, elapsed / INITIAL_SCAN_DURATION);
            int currentPoint = (int)(scanProgress * totalPoints);

            // Daha sistematik grid tarama
            currentGridStepH = currentPoint % GRID_STEPS_H;
            currentGridStepV = (currentPoint / GRID_STEPS_H) % GRID_STEPS_V;

            // Grid pozisyonunu hesapla
            double horizontalStep = MAX_SCAN_AREA_H / GRID_STEPS_H;
            double verticalStep = MAX_SCAN_AREA_V / GRID_STEPS_V;

            double targetHorizontalAngle = (currentGridStepH * horizontalStep) % 360.0;
            double targetVerticalAngle = currentGridStepV * verticalStep;

            // Antenin hareketini yumuşat
            antenna.HorizontalAngle = horizontalFilter.Update(targetHorizontalAngle);
            antenna.VerticalAngle = verticalFilter.Update(targetVerticalAngle);

            // Sinyal gücünü ölç ve filtrele
            double signal = SimulateSignalStrength(antenna, airplane, antennaLat, antennaLon, antennaAlt);
            signal = signalFilter.Update(signal);
            antenna.SignalStrength = signal;

            // En iyi sinyali güncelle
            if (signal > bestSignalStrength)
            {
                bestSignalStrength = signal;
                bestHorizontalAngle = antenna.HorizontalAngle;
                bestVerticalAngle = antenna.VerticalAngle;
                Console.WriteLine($"New best signal: {signal:F1} at H:{bestHorizontalAngle:F1}° V:{bestVerticalAngle:F1}°");
            }

            // Taramayı bitir
            if (currentPoint >= totalPoints || signal > TRACKING_THRESHOLD * 1.2)
            {
                isInitialScan = false;
                pso.Reset();
                horizontalScanArea = Math.Max(MIN_SCAN_AREA_H, 90.0);
                Console.WriteLine($"Initial scan complete. Best signal: {bestSignalStrength:F1} at H:{bestHorizontalAngle:F1}° V:{bestVerticalAngle:F1}°");
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

        /// <summary>
        /// Simulate signal strength based on antenna position relative to aircraft
        /// </summary>
        private double SimulateSignalStrength(AntennaState antenna, AirplaneState airplane,
                                           double antennaLat, double antennaLon, double antennaAlt)
        {
            GetTargetAngles(antennaLat, antennaLon, antennaAlt, airplane, out double targetH, out double targetV);

            // Açı farklarını doğru hesapla
            double hDiff = Math.Abs(((targetH - antenna.HorizontalAngle + 540.0) % 360.0) - 180.0);
            double vDiff = Math.Abs(antenna.VerticalAngle - targetV);

            // Daha dar anten paterni
            double beamAngleH = 20.0;  // Ana lob genişliği azaltıldı
            double beamAngleV = 15.0;  // Ana lob genişliği azaltıldı

            // Gaussian anten paterni
            double hGain = Math.Exp(-0.5 * Math.Pow(hDiff / (beamAngleH * 0.5), 2));
            double vGain = Math.Exp(-0.5 * Math.Pow(vDiff / (beamAngleV * 0.5), 2));

            // Mesafeye bağlı zayıflama
            double distance = CalculateDistance(antennaLat, antennaLon, antennaAlt,
                                             airplane.Latitude, airplane.Longitude, airplane.Altitude);

            double maxRange = 100000.0; // 100km maksimum menzil
            double distanceFactor = Math.Pow(Math.Max(0, 1.0 - distance / maxRange), 0.5);

            // Toplam sinyal gücü
            double signal = 100.0 * hGain * vGain * distanceFactor;

            // Minimum gürültü
            double noise = (new Random().NextDouble() - 0.5) * 0.5;

            return Math.Max(0, Math.Min(100, signal + noise));
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
            horizontalScanArea = MAX_SCAN_AREA_H;
            verticalScanArea = MAX_SCAN_AREA_V;

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