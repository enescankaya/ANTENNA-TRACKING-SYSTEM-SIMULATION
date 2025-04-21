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

        private const double MIN_SCAN_AREA = 30.0;
        private const double INITIAL_SCAN_DURATION = 15.0;
        private const double SIGNAL_LOSS_THRESHOLD = 30.0;

        private bool isInitialScan = true;
        private DateTime scanStartTime;
        private double bestSignalStrength = 0;
        private double bestHorizontalAngle = 0;
        private double bestVerticalAngle = 0;

        public bool IsInitialScan => isInitialScan;
        public double CurrentScanArea => isInitialScan ? 360.0 : Math.Max(MIN_SCAN_AREA, 360.0 * (1.0 - bestSignalStrength / 100.0));

        public AntennaController()
        {
            horizontalFilter = new KalmanFilter(0.05, 0.1, 0.05);
            verticalFilter = new KalmanFilter(0.03, 0.1, 0.05);
            signalFilter = new KalmanFilter(0.01, 0.3, 0.05);
            pso = new ParticleSwarmOptimizer(40);
        }

        public void UpdateScanningAntenna(AntennaState antenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            // Anten konumunu güncel tut
            double antennaLat = antenna.Latitude;
            double antennaLon = antenna.Longitude;
            double antennaAlt = antenna.Altitude;

            if (isInitialScan)
            {
                if (scanStartTime == default)
                {
                    scanStartTime = DateTime.Now;
                    pso.Reset();
                    bestSignalStrength = 0;
                }

                if ((DateTime.Now - scanStartTime).TotalSeconds >= INITIAL_SCAN_DURATION)
                {
                    isInitialScan = false;
                }

                var (nextH, nextV) = pso.ScanStep(
                    (h, v) => CalculateSignalStrength(h, v, antennaLat, antennaLon, antennaAlt, airplane),
                    360.0,
                    90.0
                );

                antenna.HorizontalAngle = horizontalFilter.Update(nextH % 360.0);
                antenna.VerticalAngle = verticalFilter.Update(Math.Max(0, Math.Min(90, nextV)));
                antenna.ScanAreaSize = 360.0;
            }
            else
            {
                double scanAreaH = Math.Max(MIN_SCAN_AREA, 360.0 * (1.0 - bestSignalStrength / 100.0));
                double scanAreaV = Math.Max(10.0, 90.0 * (1.0 - bestSignalStrength / 100.0));
                double centerH = bestHorizontalAngle;
                double centerV = bestVerticalAngle;

                var (offsetH, offsetV) = pso.ScanStep(
                    (h, v) =>
                    {
                        double candidateH = (centerH + h + 360.0) % 360.0;
                        double candidateV = Math.Max(0, Math.Min(90.0, centerV + v));
                        return CalculateSignalStrength(candidateH, candidateV, antennaLat, antennaLon, antennaAlt, airplane);
                    },
                    scanAreaH,
                    scanAreaV
                );

                antenna.HorizontalAngle = horizontalFilter.Update((centerH + offsetH + 360.0) % 360.0);
                antenna.VerticalAngle = verticalFilter.Update(Math.Max(0, Math.Min(90.0, centerV + offsetV)));
                antenna.ScanAreaSize = scanAreaH;

                if (antenna.SignalStrength < SIGNAL_LOSS_THRESHOLD)
                {
                    isInitialScan = true;
                    scanStartTime = DateTime.Now;
                    pso.Reset();
                }
            }

            double signal = CalculateSignalStrength(antenna.HorizontalAngle, antenna.VerticalAngle, antennaLat, antennaLon, antennaAlt, airplane);
            antenna.SignalStrength = signalFilter.Update(signal);
            if (signal > bestSignalStrength)
            {
                bestSignalStrength = signal;
                bestHorizontalAngle = antenna.HorizontalAngle;
                bestVerticalAngle = antenna.VerticalAngle;
            }
            UpdateSignalMetrics(antenna);
        }

        public void UpdateDirectionalAntenna(AntennaState antenna, AntennaState scanningAntenna, AirplaneState airplane)
        {
            if (airplane == null) return;

            // Tarama anteninin bulduğu en iyi açıya yönlen
            antenna.HorizontalAngle = horizontalFilter.Update(scanningAntenna.HorizontalAngle);
            antenna.VerticalAngle = verticalFilter.Update(scanningAntenna.VerticalAngle);

            // Anten konumunu güncel tut
            double antennaLat = antenna.Latitude;
            double antennaLon = antenna.Longitude;
            double antennaAlt = antenna.Altitude;

            antenna.SignalStrength = CalculateSignalStrength(antenna.HorizontalAngle, antenna.VerticalAngle, antennaLat, antennaLon, antennaAlt, airplane);
            UpdateSignalMetrics(antenna);
        }

        // Uçak ve anten konumuna göre sinyal gücü hesaplama
        private double CalculateSignalStrength(double antennaH, double antennaV, double antennaLat, double antennaLon, double antennaAlt, AirplaneState airplane)
        {
            double dLat = (airplane.Latitude - antennaLat) * 111000;
            double dLon = (airplane.Longitude - antennaLon) * 111000 * Math.Cos(antennaLat * Math.PI / 180.0);
            double dAlt = airplane.Altitude - antennaAlt;

            double groundDist = Math.Sqrt(dLat * dLat + dLon * dLon);
            double distance = Math.Sqrt(groundDist * groundDist + dAlt * dAlt);

            double targetH = (Math.Atan2(dLon, dLat) * 180.0 / Math.PI + 360.0) % 360.0;
            double targetV = Math.Atan2(dAlt, groundDist) * 180.0 / Math.PI;

            double hAngleDiff = Math.Abs(((antennaH - targetH + 540) % 360) - 180);
            double vAngleDiff = Math.Abs(antennaV - targetV);

            double hGain = Math.Exp(-Math.Pow(hAngleDiff / 20.0, 2));
            double vGain = Math.Exp(-Math.Pow(vAngleDiff / 10.0, 2));
            double distAtt = Math.Exp(-distance / 15000.0);

            double signal = 100.0 * hGain * vGain * distAtt;
            return Math.Max(0, Math.Min(100, signal));
        }

        private void UpdateSignalMetrics(AntennaState antenna)
        {
            antenna.RSSI = -100 + (60 * antenna.SignalStrength / 100.0);
            antenna.SNR = Math.Max(0, 30 * (antenna.SignalStrength / 100.0));
        }

        public void Reset()
        {
            horizontalFilter.Reset();
            verticalFilter.Reset();
            signalFilter.Reset();
            pso.Reset();
            isInitialScan = true;
            scanStartTime = default;
            bestSignalStrength = 0;
        }
    }
}
