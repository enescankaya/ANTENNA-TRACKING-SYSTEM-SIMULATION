using System;
using Project.Models;

namespace Project.Services
{
    public class AntennaController
    {
        private readonly KalmanFilter signalFilter;
        private readonly KalmanFilter angleFilter;
        private readonly ParticleSwarmOptimizer pso;
        private const double SIGNAL_THRESHOLD = 20.0;
        private const double MAX_VERTICAL_ANGLE = 90.0;
        private const double SCAN_STEP = 5.0;

        public AntennaController()
        {
            signalFilter = new KalmanFilter();
            angleFilter = new KalmanFilter();
            pso = new ParticleSwarmOptimizer();
        }

        public void UpdateScanningAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (antenna.SignalStrength < SIGNAL_THRESHOLD)
            {
                // Perform full scan when signal is weak
                antenna.HorizontalAngle = (antenna.HorizontalAngle + SCAN_STEP) % 360;
                antenna.VerticalAngle = Math.Min(MAX_VERTICAL_ANGLE,
                    Math.Max(0, antenna.VerticalAngle + (antenna.HorizontalAngle >= 350 ? SCAN_STEP : 0)));
            }
            else
            {
                // Use PSO for optimization when signal is strong
                double nextAngle = pso.GetNextAngle(antenna.SignalStrength);
                antenna.HorizontalAngle = angleFilter.Update(nextAngle);
            }

            // Update signal metrics
            UpdateSignalMetrics(antenna, airplane);
        }

        public void UpdateDirectionalAntenna(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate ideal angles based on airplane position
            double horizontalAngle = Math.Atan2(airplane.Longitude, airplane.Latitude) * 180 / Math.PI;
            double verticalAngle = Math.Atan2(airplane.Altitude,
                Math.Sqrt(airplane.Latitude * airplane.Latitude + airplane.Longitude * airplane.Longitude)) * 180 / Math.PI;

            // Apply Kalman filtering to smooth movement
            antenna.HorizontalAngle = angleFilter.Update(horizontalAngle);
            antenna.VerticalAngle = angleFilter.Update(verticalAngle);

            // Update signal metrics
            UpdateSignalMetrics(antenna, airplane);
        }

        private void UpdateSignalMetrics(AntennaState antenna, AirplaneState airplane)
        {
            // Calculate base RSSI based on distance and angle
            double distance = CalculateDistance(airplane);
            double angleDifference = CalculateAngleDifference(antenna, airplane);

            // Simulate RSSI (inversely proportional to distance and angle difference)
            antenna.RSSI = Math.Max(-100, -20 * Math.Log10(distance) - 0.5 * angleDifference);

            // Simulate SNR (better when more directly aligned)
            antenna.SNR = Math.Max(0, 30 - (angleDifference / 2));

            // Combined signal strength (normalized to 0-100 range)
            double rawSignalStrength = ((antenna.RSSI + 100) / 100.0 * 0.7 + antenna.SNR / 30.0 * 0.3) * 100;
            antenna.SignalStrength = signalFilter.Update(rawSignalStrength);
        }

        private double CalculateDistance(AirplaneState airplane)
        {
            return Math.Sqrt(
                airplane.Latitude * airplane.Latitude +
                airplane.Longitude * airplane.Longitude +
                airplane.Altitude * airplane.Altitude);
        }

        private double CalculateAngleDifference(AntennaState antenna, AirplaneState airplane)
        {
            double targetHAngle = Math.Atan2(airplane.Longitude, airplane.Latitude) * 180 / Math.PI;
            double hDiff = Math.Abs(((antenna.HorizontalAngle - targetHAngle + 180) % 360) - 180);

            double targetVAngle = Math.Atan2(airplane.Altitude,
                Math.Sqrt(airplane.Latitude * airplane.Latitude + airplane.Longitude * airplane.Longitude)) * 180 / Math.PI;
            double vDiff = Math.Abs(antenna.VerticalAngle - targetVAngle);

            return Math.Sqrt(hDiff * hDiff + vDiff * vDiff);
        }

        public void Reset()
        {
            signalFilter.Reset();
            angleFilter.Reset();
        }
    }
}
