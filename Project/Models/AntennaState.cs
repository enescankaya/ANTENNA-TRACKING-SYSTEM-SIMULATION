namespace Project.Models
{
    public class AntennaState
    {
        // Açısal pozisyon
        public double HorizontalAngle { get; set; }  // 0-360 derece
        public double VerticalAngle { get; set; }    // 0-90 derece

        // Sinyal metrikleri
        public double SignalStrength { get; set; }   // 0-100 normalize edilmiş
        public double RSSI { get; set; }             // dBm cinsinden, tipik -100 ile -40 arası
        public double SNR { get; set; }              // dB cinsinden, 0-30 arası

        // GPS konumu
        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public double Altitude { get; set; }

        // Servo kontrol meta verileri
        public bool IsScanning { get; set; }         // Tarama modunda mı?
        public double ScanAreaSize { get; set; }     // Mevcut tarama alanı boyutu (derece)
        public double TargetAngle { get; set; }      // Hedeflenen açı
        public double LastSignalUpdate { get; set; } // Son sinyal güncellemesi zamanı
    }
}
