using System.Windows.Media;

namespace Project.Services
{
    public static class ColorHelper
    {
        public static Color LerpColor(Color start, Color end, double amount)
        {
            return Color.FromArgb(
                (byte)(start.A + (end.A - start.A) * amount),
                (byte)(start.R + (end.R - start.R) * amount),
                (byte)(start.G + (end.G - start.G) * amount),
                (byte)(start.B + (end.B - start.B) * amount)
            );
        }
    }
}
