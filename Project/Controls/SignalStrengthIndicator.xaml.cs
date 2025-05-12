using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;

namespace Project.Controls
{
    public partial class SignalStrengthIndicator : UserControl
    {
        private readonly DoubleAnimation heightAnimation;

        public SignalStrengthIndicator()
        {
            InitializeComponent();

            heightAnimation = new DoubleAnimation
            {
                Duration = TimeSpan.FromMilliseconds(500),
                EasingFunction = new ExponentialEase { EasingMode = EasingMode.EaseOut }
            };
        }

        // RSSI: -120 dBm (çok kötü) ile -40 dBm (çok iyi) arasında
        // SNR: -20 dB (çok kötü) ile +40 dB (çok iyi) arasında
        public void UpdateSignal(double rssi, double snr)
        {
            double normalizedRssi = (rssi + 120) / 80.0; // -120 to -40 -> 0 to 1
            normalizedRssi = Math.Max(0, Math.Min(1, normalizedRssi));

            double height = normalizedRssi * SignalContainer.ActualHeight;
            SignalFill.Height = height;

            SignalValue.Text = $"{rssi:F1} dBm";
            SnrValue.Text = $"SNR: {snr:F1} dB";

            // Update glow effect color based on signal strength
            var effect = SignalFill.Effect as DropShadowEffect;
            if (effect != null)
            {
                if (normalizedRssi < 0.3)
                    effect.Color = Colors.Red;
                else if (normalizedRssi < 0.7)
                    effect.Color = Colors.Yellow;
                else
                    effect.Color = Colors.Lime;
            }
        }
    }
}
