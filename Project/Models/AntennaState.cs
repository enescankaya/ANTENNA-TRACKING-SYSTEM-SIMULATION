namespace Project.Models
{
    public class AntennaState
    {
        public double HorizontalAngle { get; set; }
        public double VerticalAngle { get; set; }
        public double SignalStrength { get; set; }
        public double RSSI { get; set; }
        public double SNR { get; set; }

        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public double Altitude { get; set; }
    }
}
