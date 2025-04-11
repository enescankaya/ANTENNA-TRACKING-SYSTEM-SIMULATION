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

        // Signal thresholds - adjusted for realistic values
        private const double CRITICAL_SIGNAL_THRESHOLD = -85.0;  // dBm
        private const double GOOD_SIGNAL_THRESHOLD = -65.0;     // dBm
        private const double EXCELLENT_SIGNAL_THRESHOLD = 75.0; // dBm

        // Scan area parameters
        private const double MAX_HORIZONTAL_ANGLE = 360.0;
        private const double MAX_VERTICAL_ANGLE = 90.0;
        private const double MIN_SCAN_AREA = 30.0;  // Minimum scan area in degrees
        private const double SCAN_STEP = 5.0;
        private double currentScanArea = 360.0;     // Start with full scan
        private const double SCAN_AREA_REDUCTION = 0.8; // 20% reduction each time

        // State variables
        private bool isFullScan = true;
        private double bestSignalStrength = 0;
        private double bestHorizontalAngle = 0;
        private double bestVerticalAngle = 0;
        private DateTime lastDirectionalUpdate = DateTime.Now;
        private const int DIRECTIONAL_UPDATE_INTERVAL = 500; // ms
        private double lastSignalStrength = 0;
        private const int SIGNAL_HISTORY_SIZE = 10;
        private readonly Queue<double> signalHistory = new Queue<double>();
        private DateTime lastScanTime = DateTime.Now;
        private const int SCAN_INTERVAL = 50; // Reduced from 100ms for faster updates
        private bool isInitialScan = true;    // Track initial scan state
        private const double SIGNAL_LOSS_THRESHOLD = 30.0; // Signal threshold for reset

        public AntennaController()
        {
            // Initialize Kalman filters with optimized parameters
            horizontalFilter = new KalmanFilter(
                processNoise: 0.05,  // Increased for more responsive tracking
                measurementNoise: 0.1,
                dt: 0.05
            );

            verticalFilter = new KalmanFilter(
                processNoise: 0.03,  // Increased for more responsive tracking
                measurementNoise: 0.1,
                dt: 0.05
            );

            // More aggressive signal filtering
            signalFilter = new KalmanFilter(
                processNoise: 0.01,
                measurementNoise: 0.3,
                dt: 0.05
            );

            pso = new ParticleSwarmOptimizer(40); // Increased particle count
        }

        public void UpdateScanningAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            if ((DateTime.Now - lastScanTime).TotalMilliseconds < SCAN_INTERVAL) return;
            lastScanTime = DateTime.Now;

            // Calculate real target angles
            double targetH = CalculateTargetHorizontalAngle(antenna, airplane);
            double targetV = CalculateTargetVerticalAngle(antenna, airplane);

            // Calculate current signal strength
            double currentSignal = CalculateSignalStrength(antenna, airplane);
            double filteredSignal = signalFilter.Update(currentSignal);
            antenna.SignalStrength = filteredSignal;

            // Check if we need to reset to full scan
            if (filteredSignal < SIGNAL_LOSS_THRESHOLD || isInitialScan)
            {
                currentScanArea = 360.0;
                pso.Reset();
                isInitialScan = filteredSignal < SIGNAL_LOSS_THRESHOLD;
            }

            // PSO search with dynamic radius
            double searchRadius = currentScanArea / 2;
            var (nextH, nextV) = pso.GetNextPosition(
                filteredSignal,
                searchRadius,
                targetH,
                targetV
            );

            // Apply Kalman filtering for smooth movement
            antenna.HorizontalAngle = horizontalFilter.Update(nextH);
            antenna.VerticalAngle = verticalFilter.Update(nextV);

            // Update metrics and history
            UpdateSignalMetrics(antenna, airplane);
            UpdateSignalHistory(filteredSignal);
            UpdateBestPosition(antenna);

            // Adapt scan area based on signal quality
            AdaptScanArea(filteredSignal);
        }

        private void AdaptScanArea(double signalStrength)
        {
            if (signalStrength > EXCELLENT_SIGNAL_THRESHOLD)
            {
                currentScanArea = Math.Max(MIN_SCAN_AREA, currentScanArea * 0.9);
            }
            else if (signalStrength < SIGNAL_LOSS_THRESHOLD)
            {
                currentScanArea = Math.Min(360.0, currentScanArea * 1.5);
                pso.IncreaseSearchArea();
            }
        }

        public void UpdateDirectionalAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            if ((DateTime.Now - lastDirectionalUpdate).TotalMilliseconds < DIRECTIONAL_UPDATE_INTERVAL) return;

            // Always calculate real target angles
            double targetH = CalculateTargetHorizontalAngle(antenna, airplane);
            double targetV = CalculateTargetVerticalAngle(antenna, airplane);

            // Calculate current signal and get best known position
            double currentSignal = CalculateSignalStrength(antenna, airplane);
            double bestH = bestHorizontalAngle;
            double bestV = bestVerticalAngle;

            // Dynamic position mixing based on signal quality
            double mixFactor = CalculateInterpolationFactor(currentSignal);
            double finalH = (bestH * mixFactor) + (targetH * (1 - mixFactor));
            double finalV = (bestV * mixFactor) + (targetV * (1 - mixFactor));

            // Apply Kalman filtering for smooth movement
            antenna.HorizontalAngle = horizontalFilter.Update((finalH + 360.0) % 360.0);
            antenna.VerticalAngle = verticalFilter.Update(Math.Max(0, Math.Min(90, finalV)));

            // Update signal metrics
            UpdateSignalMetrics(antenna, airplane);
            lastDirectionalUpdate = DateTime.Now;
        }

        private double CalculateInterpolationFactor(double signalStrength)
        {
            // Adjust factor based on signal strength
            if (signalStrength > 80) return 0.9;     // Very strong signal - trust best position
            if (signalStrength > 60) return 0.7;     // Good signal
            if (signalStrength > 40) return 0.5;     // Medium signal
            return 0.3;                              // Weak signal - favor actual position
        }

        private void AdjustScanArea(double signalStrength)
        {
            if (signalStrength > 80 && currentScanArea > MIN_SCAN_AREA)
            {
                currentScanArea *= SCAN_AREA_REDUCTION;
            }
            else if (signalStrength < 40)
            {
                currentScanArea = Math.Min(360.0, currentScanArea * 1.5);
            }
        }

        private double CalculateSignalStrength(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate angles to target
            double targetH = CalculateTargetHorizontalAngle(antenna, airplane);
            double targetV = CalculateTargetVerticalAngle(antenna, airplane);

            // Calculate angle differences
            double hAngleDiff = Math.Abs(((antenna.HorizontalAngle - targetH + 540) % 360) - 180);
            double vAngleDiff = Math.Abs(antenna.VerticalAngle - targetV);

            // Distance-based attenuation
            double distance = CalculateDistance(antenna, airplane);
            double maxRange = 10000.0; // 10km max range
            double distanceAttenuation = Math.Exp(-distance / maxRange);

            // Angle-based attenuation
            double maxAngleError = 180.0;
            double hAngleAttenuation = Math.Exp(-Math.Pow(hAngleDiff / maxAngleError, 2));
            double vAngleAttenuation = Math.Exp(-Math.Pow(vAngleDiff / 90.0, 2));

            // Combined signal strength (0-100)
            double signal = 100.0 * distanceAttenuation * hAngleAttenuation * vAngleAttenuation;

            // Add realistic noise
            Random random = new Random();
            double noise = (random.NextDouble() - 0.5) * 2.0 * (1.0 - signal / 100.0); // More noise at lower signal levels
            signal = Math.Max(0, Math.Min(100, signal + noise));

            return signal;
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

            double avgRecent = signalHistory.Skip(SIGNAL_HISTORY_SIZE / 2).Average();
            double avgOlder = signalHistory.Take(SIGNAL_HISTORY_SIZE / 2).Average();
            return avgRecent < avgOlder * 0.9; // 10% degradation threshold
        }

        private void IncreaseSearchArea()
        {
            currentScanArea = Math.Min(360.0, currentScanArea * 1.5);
            pso.Reset();
        }

        private void UpdateSignalMetrics(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate raw values with realistic noise
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
            if (rssi < -80) return 0.6;  // Weak signal - consider SNR more
            return 0.7; // Medium signal - balanced weight
        }

        private double NormalizeRSSI(double rssi)
        {
            return (rssi + 100) / 60 * 100; // Convert -100 to -40 dBm to 0-100
        }

        private double NormalizeSNR(double snr)
        {
            return snr / 30 * 100; // Convert 0-30 dB to 0-100
        }

        private void ResetScan()
        {
            currentScanArea = 360.0;
            bestSignalStrength = 0;
            pso.Reset();
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
        }

        private double CalculateTargetHorizontalAngle(AntennaState antenna, AirplaneState airplane)
        {
            double dX = (airplane.Longitude - antenna.Longitude) * Math.Cos(antenna.Latitude * Math.PI / 180.0);
            double dY = airplane.Latitude - antenna.Latitude;
            return (Math.Atan2(dX, dY) * 180.0 / Math.PI + 360.0) % 360.0;
        }

        private double CalculateTargetVerticalAngle(AntennaState antenna, AirplaneState airplane)
        {
            double distance = CalculateDistance(antenna, airplane);
            return Math.Atan2(airplane.Altitude, distance) * 180.0 / Math.PI;
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
            return Math.Atan2(airplane.Altitude, distance) * 180.0 / Math.PI;
        }

        private double CalculateRSSI(AntennaState antenna, AirplaneState airplane)
        {
            // Free space path loss formula for RSSI calculation
            double distance = CalculateDistance(antenna, airplane);
            double frequency = 2.4e9; // 2.4 GHz
            double c = 3e8; // speed of light
            double lambda = c / frequency;

            // Free space path loss
            double FSPL = 20 * Math.Log10(distance) + 20 * Math.Log10(frequency) - 147.55;

            // Include antenna gain and angle effect
            double antennaGain = 5.0; // dBi
            double angleEffect = Math.Cos(Math.Abs(antenna.HorizontalAngle - CalculateHorizontalAngle(airplane)) * Math.PI / 180);

            // Transmit power (typical for drones)
            double txPower = 20; // dBm

            // Final RSSI calculation
            double rssi = txPower + antennaGain - FSPL + (10 * Math.Log10(Math.Max(0.1, angleEffect)));

            return Math.Max(-100, Math.Min(-40, rssi));
        }

        private double CalculateSNR(AntennaState antenna, AirplaneState airplane)
        {
            double rssi = CalculateRSSI(antenna, airplane);
            double noiseFloor = -90; // Typical noise floor in dBm
            double snr = rssi - noiseFloor;

            // Include angle effect
            double angleFactor = Math.Cos(Math.Abs(antenna.HorizontalAngle - CalculateHorizontalAngle(airplane)) * Math.PI / 180);
            snr *= Math.Max(0.1, angleFactor);

            return Math.Max(0, Math.Min(30, snr)); // Limit to 0-30 dB range
        }

        private double CalculateDistance(AntennaState antenna, AirplaneState airplane)
        {
            return Math.Sqrt(
                Math.Pow((airplane.Latitude - antenna.Latitude) * 111000, 2) +
                Math.Pow((airplane.Longitude - antenna.Longitude) * 111000 * Math.Cos(antenna.Latitude * Math.PI / 180), 2) +
                Math.Pow(airplane.Altitude, 2)
            );
        }

        private void UpdateBestPosition(AntennaState antenna)
        {
            // Update best position if current signal is stronger
            if (antenna.SignalStrength > bestSignalStrength)
            {
                bestSignalStrength = antenna.SignalStrength;
                bestHorizontalAngle = antenna.HorizontalAngle;
                bestVerticalAngle = antenna.VerticalAngle;

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
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
            pso.Reset();
            isFullScan = true;
            currentScanArea = 360.0;
            bestSignalStrength = 0;
        }
    }
}