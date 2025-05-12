using System;
using System.Windows;
using System.Windows.Threading;

namespace Project.Views
{
    public partial class SplashScreen : Window
    {
        private readonly Random random = new Random();
        private readonly double totalDuration;
        private readonly double progressIncrement;
        private DispatcherTimer timer;
        private double progress = 0;
        private readonly string[] loadingTexts = new[]
        {
            "Initializing components...",
            "Loading modules...",
            "Configuring system...",
            "Preparing interface...",
            "Almost ready..."
        };

        public SplashScreen()
        {
            InitializeComponent();

            // 10-15 saniye arası random süre belirle
            totalDuration = random.Next(10000, 15001); // 10000-15000 ms arası
            progressIncrement = 100.0 / (totalDuration / 30.0); // Her tick için artış miktarı

            InitializeTimer();
        }

        private void InitializeTimer()
        {
            timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(30) // Daha smooth animasyon için
            };

            timer.Tick += Timer_Tick;
            timer.Start();
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            try
            {
                progress += progressIncrement;

                if (progress >= 100)
                {
                    timer.Stop();
                    progress = 100;
                    LoadingText.Text = "Ready to launch...";

                    // Son mesajı göstermek için kısa bir bekleme
                    DispatcherTimer closeTimer = new DispatcherTimer
                    {
                        Interval = TimeSpan.FromSeconds(1)
                    };
                    closeTimer.Tick += (s, args) =>
                    {
                        closeTimer.Stop();
                        Close();
                    };
                    closeTimer.Start();
                    return;
                }

                // Loading mesajlarını güncelle
                if (progress < 20) LoadingText.Text = loadingTexts[0];
                else if (progress < 40) LoadingText.Text = loadingTexts[1];
                else if (progress < 60) LoadingText.Text = loadingTexts[2];
                else if (progress < 80) LoadingText.Text = loadingTexts[3];
                else LoadingText.Text = loadingTexts[4];

                LoadingBar.Value = progress;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error in splash screen: {ex.Message}");
                Close();
            }
        }
    }
}
