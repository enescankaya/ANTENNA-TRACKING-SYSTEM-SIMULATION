namespace Project.Models
{
    public class AirplaneState
    {
        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public double Altitude { get; set; }
        public double Heading { get; set; }
        public double Roll { get; set; }
        public double Pitch { get; set; }
        public double Throttle { get; set; }
        public double Battery { get; set; }
        public double GroundSpeed { get; set; }  // m/s cinsinden hız
        public double AirSpeed { get; set; }     // m/s cinsinden hız
        public double ClimbRate { get; set; }    // m/s cinsinden tırmanış hızı
    }
}
