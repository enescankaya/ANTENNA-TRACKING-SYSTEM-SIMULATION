using System;
using System.Collections.Generic;
using System.Linq;
using Project.Models;

namespace Project.Services
{
    public class AntennaController
    {
        private readonly KalmanFilter horizontalFilter;
        private readonly KalmanFilter verticalFilter;
        private readonly KalmanFilter signalFilter;
        private readonly ParticleSwarmOptimizer pso;

        // Signal thresholds - corrected for realistic values
        private const double CRITICAL_SIGNAL_THRESHOLD = -85.0;  // dBm
        private const double GOOD_SIGNAL_THRESHOLD = -65.0;     // dBm
        private const double EXCELLENT_SIGNAL_THRESHOLD = -55.0; // dBm - fixed from 75.0

        // Scan area parameters
        private const double MAX_HORIZONTAL_ANGLE = 360.0;
        private const double MAX_VERTICAL_ANGLE = 90.0;
        private const double MIN_SCAN_AREA = 30.0;  // Minimum scan area in degrees
        private const double SCAN_STEP = 5.0;
        private double currentScanArea = 360.0;     // Start with full scan
        private const double SCAN_AREA_REDUCTION = 0.9; // 10% reduction each time (less aggressive)

        // State variables
        private bool isFullScan = true;
        private double bestSignalStrength = 0;
        private double bestHorizontalAngle = 180.0; // Start in middle position
        private double bestVerticalAngle = 45.0;    // Start in middle position
        private DateTime lastDirectionalUpdate = DateTime.Now;
        private const int DIRECTIONAL_UPDATE_INTERVAL = 50; // ms - faster updates
        private double lastSignalStrength = 0;
        private const int SIGNAL_HISTORY_SIZE = 10;
        private readonly Queue<double> signalHistory = new Queue<double>();
        private DateTime lastScanTime = DateTime.Now;
        private const int SCAN_INTERVAL = 25; // Reduced from 50ms for even faster updates
        private bool isInitialScan = true;    // Track initial scan state
        private const double SIGNAL_LOSS_THRESHOLD = 20.0; // Signal threshold for reset

        public AntennaController()
        {
            // Initialize Kalman filters with more responsive parameters
            horizontalFilter = new KalmanFilter(
                processNoise: 0.1,  // Increased for more responsiveness to target movement
                measurementNoise: 0.1,
                dt: 0.05
            );

            verticalFilter = new KalmanFilter(
                processNoise: 0.08, // Increased for better vertical tracking
                measurementNoise: 0.1,
                dt: 0.05
            );

            // More balanced signal filtering
            signalFilter = new KalmanFilter(
                processNoise: 0.03, // Increased to better follow signal changes
                measurementNoise: 0.2, // Decreased to trust measurements more
                dt: 0.05
            );

            pso = new ParticleSwarmOptimizer(40); // Increased particle count
        }

        private double CalculateSignalStrength(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return 0;

            // Calculate 3D distance in meters
            double distance = CalculateDistance(antenna, airplane);

            // Calculate target angles
            double targetH = CalculateTargetHorizontalAngle(antenna, airplane);
            double targetV = CalculateTargetVerticalAngle(antenna, airplane);

            // Angular difference calculation - shortest angle
            double hAngleDiff = Math.Abs(NormalizeAngle(antenna.HorizontalAngle - targetH));
            double vAngleDiff = Math.Abs(antenna.VerticalAngle - targetV);

            // Beamwidth effects (3dB beamwidth typical values)
            double horizontalBeamwidth = 30.0; // degrees - narrower beam
            double verticalBeamwidth = 15.0; // degrees - narrower beam

            // Antenna gain pattern simulation (Gaussian)
            double hGain = Math.Exp(-0.5 * Math.Pow(hAngleDiff / (horizontalBeamwidth / 2), 2));
            double vGain = Math.Exp(-0.5 * Math.Pow(vAngleDiff / (verticalBeamwidth / 2), 2));

            // Combined antenna gain
            double antennaGain = 25.0 * hGain * vGain; // dBi max gain - increased gain

            // Free space path loss calculation
            double frequency = 2.4e9; // 2.4 GHz
            double wavelength = 3e8 / frequency;
            double fspl = 20 * Math.Log10(distance) + 20 * Math.Log10(frequency) - 147.55;

            // Atmospheric effects
            double atmosphericLoss = 0.001 * distance; // dB loss (simplified)

            // Final RSSI calculation including all factors
            double rxPower = 20.0 // Transmit power (dBm)
                         + antennaGain
                         - fspl
                         - atmosphericLoss;

            // Convert to 0-100 scale with realistic noise
            double normalizedSignal = (rxPower + 100) / 60 * 100;

            // Add environmental noise
            Random random = new Random();
            double snr = Math.Max(1, normalizedSignal / 10); // Signal-to-noise ratio, min 1
            double noise = (random.NextDouble() - 0.5) * 2.0 * (10.0 / snr);

            return Math.Max(0, Math.Min(100, normalizedSignal + noise));
        }

        private double NormalizeAngle(double angle)
        {
            angle = (angle % 360 + 360) % 360;
            if (angle > 180)
                angle -= 360;
            return angle;
        }

        private double CalculateTargetHorizontalAngle(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate delta in cartesian coordinates accounting for longitude distortion
            double deltaLon = (airplane.Longitude - antenna.Longitude) *
                             Math.Cos((antenna.Latitude + airplane.Latitude) * Math.PI / 360.0);
            double deltaLat = airplane.Latitude - antenna.Latitude;

            // Calculate angle in radians
            double angle = Math.Atan2(deltaLon, deltaLat);

            // Convert to degrees and normalize to 0-360
            double degrees = (angle * 180.0 / Math.PI + 360.0) % 360.0;

            return degrees;
        }

        private double CalculateTargetVerticalAngle(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate horizontal distance in meters accounting for Earth's curvature
            double horizontalDistance = Math.Sqrt(
                Math.Pow((airplane.Latitude - antenna.Latitude) * 111000, 2) +
                Math.Pow((airplane.Longitude - antenna.Longitude) * 111000 *
                      Math.Cos(antenna.Latitude * Math.PI / 180), 2)
            );

            // Height difference (meters)
            double heightDifference = airplane.Altitude;

            // Calculate vertical angle using arctangent
            double angle = Math.Atan2(heightDifference, horizontalDistance) * 180.0 / Math.PI;

            // Limit to 0-90 degree range
            return Math.Max(0, Math.Min(90, angle));
        }

        public void UpdateScanningAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            if ((DateTime.Now - lastScanTime).TotalMilliseconds < SCAN_INTERVAL) return;
            lastScanTime = DateTime.Now;

            // Calculate target angles
            double targetH = CalculateTargetHorizontalAngle(antenna, airplane);
            double targetV = CalculateTargetVerticalAngle(antenna, airplane);

            // Calculate current signal strength and filter it
            double currentSignal = CalculateSignalStrength(antenna, airplane);
            double filteredSignal = signalFilter.Update(currentSignal);
            antenna.SignalStrength = filteredSignal;

            // Reset if signal is lost or during initial scan
            if (filteredSignal < SIGNAL_LOSS_THRESHOLD || isInitialScan)
            {
                ResetScan();
                isInitialScan = false;
                currentScanArea = 360.0;
                pso.IncreaseSearchArea(); // Force particles to spread out
            }

            // Calculate dynamic search radius based on signal quality
            double searchRadius = CalculateSearchRadius(filteredSignal);

            // Use PSO to determine next position
            var (nextH, nextV) = pso.GetNextPosition(
                filteredSignal,
                searchRadius,
                targetH,
                targetV
            );

            // Apply Kalman filtering for smooth movement
            // Calculate shortest rotation path for horizontal angle
            double currentH = antenna.HorizontalAngle;
            double diffH = NormalizeAngle(nextH - currentH);
            double smoothH = currentH + diffH;

            antenna.HorizontalAngle = horizontalFilter.Update(smoothH);
            antenna.VerticalAngle = verticalFilter.Update(nextV);

            // Update metrics and state
            UpdateSignalMetrics(antenna, airplane);
            UpdateSignalHistory(filteredSignal);
            UpdateBestPosition(antenna);

            // Adjust scan area dynamically
            AdjustScanArea(filteredSignal);
        }

        private double CalculateSearchRadius(double signalStrength)
        {
            // More dynamic search radius calculation
            if (signalStrength > 80)
                return MIN_SCAN_AREA / 4;  // Tight search for strong signals
            else if (signalStrength > 60)
                return MIN_SCAN_AREA / 2;      // Standard search for good signals
            else if (signalStrength > 40)
                return Math.Max(30.0, currentScanArea / 4);  // Wider search for weak signals
            else
                return Math.Max(60.0, currentScanArea * 0.3);  // Very wide search for poor signals
        }

        public void UpdateDirectionalAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            if ((DateTime.Now - lastDirectionalUpdate).TotalMilliseconds < DIRECTIONAL_UPDATE_INTERVAL) return;
            lastDirectionalUpdate = DateTime.Now;

            // Calculate actual target angles
            double targetH = CalculateTargetHorizontalAngle(antenna, airplane);
            double targetV = CalculateTargetVerticalAngle(antenna, airplane);

            // Get current signal strength
            double currentSignal = CalculateSignalStrength(antenna, airplane);

            // Dynamic mixing with more aggressive following of actual target position
            double mixFactor = CalculateInterpolationFactor(currentSignal);
            double finalH = (bestHorizontalAngle * mixFactor) + (targetH * (1 - mixFactor));
            double finalV = (bestVerticalAngle * mixFactor) + (targetV * (1 - mixFactor));

            // Calculate shortest path for horizontal angle rotation
            double currentH = antenna.HorizontalAngle;
            double diffH = NormalizeAngle(finalH - currentH);
            double smoothH = currentH + diffH;

            // Apply Kalman filtering for smooth motion
            antenna.HorizontalAngle = horizontalFilter.Update(smoothH);
            antenna.VerticalAngle = verticalFilter.Update(Math.Max(0, Math.Min(90, finalV)));

            // Update metrics
            UpdateSignalMetrics(antenna, airplane);
        }

        private double CalculateInterpolationFactor(double signalStrength)
        {
            // More aggressive tracking adjustment based on signal quality
            if (signalStrength > 80) return 0.9;     // Strong signal - trust best position but still follow
            if (signalStrength > 60) return 0.7;     // Good signal - balanced approach
            if (signalStrength > 40) return 0.5;     // Medium signal - favor actual position more
            return 0.3;                              // Weak signal - mostly follow calculated position
        }

        private void AdjustScanArea(double signalStrength)
        {
            // More gradual scan area adjustment
            if (signalStrength > 80 && currentScanArea > MIN_SCAN_AREA)
            {
                currentScanArea = Math.Max(MIN_SCAN_AREA, currentScanArea * SCAN_AREA_REDUCTION);
            }
            else if (signalStrength > 60 && currentScanArea > MIN_SCAN_AREA * 2)
            {
                currentScanArea = Math.Max(MIN_SCAN_AREA * 2, currentScanArea * 0.95);
            }
            else if (signalStrength < 40)
            {
                // Expand search area more aggressively for weak signals
                currentScanArea = Math.Min(360.0, currentScanArea * 1.2);
            }
            else if (signalStrength < 20)
            {
                // Reset to full scan for very weak signals
                currentScanArea = 360.0;
            }
        }

        private double CalculateRSSI(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate target angles
            double targetH = CalculateTargetHorizontalAngle(antenna, airplane);
            double targetV = CalculateTargetVerticalAngle(antenna, airplane);

            // Get shortest angular difference for horizontal
            double horizontalDiff = Math.Abs(NormalizeAngle(antenna.HorizontalAngle - targetH));
            double verticalDiff = Math.Abs(antenna.VerticalAngle - targetV);

            // Calculate 3D distance
            double distance = CalculateDistance(antenna, airplane);

            // Free space path loss calculation with frequency
            double frequency = 2.4e9; // 2.4 GHz
            double c = 3e8; // speed of light
            double lambda = c / frequency;

            // Free space path loss
            double FSPL = 20 * Math.Log10(distance) + 20 * Math.Log10(frequency) - 147.55;

            // Antenna gain based on angular difference
            double antennaGain = 15.0; // dBi max gain
            double horizontalGainFactor = Math.Exp(-Math.Pow(horizontalDiff / 30.0, 2));
            double verticalGainFactor = Math.Exp(-Math.Pow(verticalDiff / 15.0, 2));
            double effectiveGain = antennaGain * horizontalGainFactor * verticalGainFactor;

            // Transmit power
            double txPower = 20; // dBm

            // Calculate final RSSI
            double rssi = txPower + effectiveGain - FSPL;

            return Math.Max(-100, Math.Min(-40, rssi));
        }

        private double CalculateSNR(AntennaState antenna, AirplaneState airplane)
        {
            double rssi = CalculateRSSI(antenna, airplane);
            double noiseFloor = -95; // Typical noise floor in dBm
            double snr = rssi - noiseFloor;

            // Get angular alignment factor
            double targetH = CalculateTargetHorizontalAngle(antenna, airplane);
            double horizontalDiff = Math.Abs(NormalizeAngle(antenna.HorizontalAngle - targetH));
            double angleFactor = Math.Exp(-Math.Pow(horizontalDiff / 60.0, 2));
            snr *= Math.Max(0.3, angleFactor);

            return Math.Max(0, Math.Min(30, snr)); // Limit to 0-30 dB range
        }

        private double CalculateDistance(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate 3D distance accounting for Earth's curvature
            return Math.Sqrt(
                Math.Pow((airplane.Latitude - antenna.Latitude) * 111000, 2) +
                Math.Pow((airplane.Longitude - antenna.Longitude) * 111000 *
                      Math.Cos(antenna.Latitude * Math.PI / 180), 2) +
                Math.Pow(airplane.Altitude, 2)
            );
        }

        private void UpdateSignalHistory(double signal)
        {
            signalHistory.Enqueue(signal);
            if (signalHistory.Count > SIGNAL_HISTORY_SIZE)
                signalHistory.Dequeue();
        }

        private bool IsSignalDegrading()
        {
            if (signalHistory.Count < SIGNAL_HISTORY_SIZE) return false;

            // Use more aggressive threshold for signal degradation detection
            double avgRecent = signalHistory.Skip(SIGNAL_HISTORY_SIZE / 2).Average();
            double avgOlder = signalHistory.Take(SIGNAL_HISTORY_SIZE / 2).Average();
            return avgRecent < avgOlder * 0.85; // 15% degradation threshold (more sensitive)
        }

        private void IncreaseSearchArea()
        {
            // More aggressive expansion when signal is degrading
            currentScanArea = Math.Min(360.0, currentScanArea * 1.5);
            pso.IncreaseSearchArea();
        }

        private void UpdateSignalMetrics(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate raw values
            double rawRSSI = CalculateRSSI(antenna, airplane);
            double rawSNR = CalculateSNR(antenna, airplane);

            // Apply Kalman filtering
            double filteredRSSI = signalFilter.Update(rawRSSI);
            double filteredSNR = signalFilter.Update(rawSNR);

            // Store filtered values
            antenna.RSSI = filteredRSSI;
            antenna.SNR = filteredSNR;

            // Weighted signal strength calculation
            double normalizedRSSI = NormalizeRSSI(filteredRSSI);
            double normalizedSNR = NormalizeSNR(filteredSNR);

            // Dynamic weighting based on signal conditions
            double rssiWeight = GetRSSIWeight(filteredRSSI);
            double snrWeight = 1 - rssiWeight;

            double combinedSS = (normalizedRSSI * rssiWeight + normalizedSNR * snrWeight);
            antenna.SignalStrength = Math.Max(0, Math.Min(100, combinedSS));
        }

        private double GetRSSIWeight(double rssi)
        {
            // Adjust RSSI weight based on signal strength
            if (rssi > -60) return 0.8;  // Strong signal - trust RSSI more
            if (rssi < -80) return 0.5;  // Weak signal - consider SNR more
            return 0.7; // Medium signal - balanced weight
        }

        private double NormalizeRSSI(double rssi)
        {
            // Convert -100 to -40 dBm to 0-100 scale
            return Math.Min(100, Math.Max(0, (rssi + 100) / 60 * 100));
        }

        private double NormalizeSNR(double snr)
        {
            // Convert 0-30 dB to 0-100 scale
            return Math.Min(100, Math.Max(0, snr / 30 * 100));
        }

        private void ResetScan()
        {
            // Complete reset of scan state
            currentScanArea = 360.0;
            bestSignalStrength = 0;
            pso.Reset();
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
            signalHistory.Clear();
        }

        private void UpdateBestPosition(AntennaState antenna)
        {
            // More aggressive best position update
            if (antenna.SignalStrength > bestSignalStrength * 0.98) // Allow minor improvement to count
            {
                // Smooth update of best position
                double updateWeight = Math.Min(1.0, (antenna.SignalStrength - bestSignalStrength) / 20.0 + 0.3);

                if (antenna.SignalStrength > bestSignalStrength)
                {
                    bestSignalStrength = antenna.SignalStrength;
                }

                // Weighted update of best angles
                double oldWeight = 1 - updateWeight;

                // Calculate shortest path for horizontal angle update
                double diffH = NormalizeAngle(antenna.HorizontalAngle - bestHorizontalAngle);
                bestHorizontalAngle = NormalizeAngle(bestHorizontalAngle + diffH * updateWeight);

                // Update vertical angle with weighting
                bestVerticalAngle = bestVerticalAngle * oldWeight + antenna.VerticalAngle * updateWeight;
                bestVerticalAngle = Math.Max(0, Math.Min(90, bestVerticalAngle));

                // Adjust search area when finding better signal
                if (bestSignalStrength > 60 && currentScanArea > MIN_SCAN_AREA)
                {
                    currentScanArea *= SCAN_AREA_REDUCTION;
                    pso.UpdateSearchArea(bestSignalStrength, bestHorizontalAngle, bestVerticalAngle);
                }
            }
            else if (IsSignalDegrading())
            {
                // If signal is degrading, increase search area
                IncreaseSearchArea();
            }
        }

        public void Reset()
        {
            // Complete system reset
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
            pso.Reset();
            isFullScan = true;
            isInitialScan = true;
            currentScanArea = 360.0;
            bestSignalStrength = 0;
            bestHorizontalAngle = 180.0;
            bestVerticalAngle = 45.0;
            signalHistory.Clear();
        }
    }
}