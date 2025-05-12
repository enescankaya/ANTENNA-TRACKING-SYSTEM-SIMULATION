using System;
using System.Windows;
using System.Windows.Threading;

namespace Project.Views
{
    public partial class SplashScreen : Window
    {
        private Random random = new Random();
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
            InitializeTimer();
        }

        private void InitializeTimer()
        {
            timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(30)
            };

            timer.Tick += Timer_Tick;
            timer.Start();
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            try
            {
                progress += random.NextDouble() * 1.5;

                if (progress >= 100)
                {
                    timer.Stop();
                    progress = 100;
                    LoadingText.Text = "Ready to launch...";

                    // Kısa bir bekleme süresi sonra kapat
                    DispatcherTimer closeTimer = new DispatcherTimer
                    {
                        Interval = TimeSpan.FromSeconds(1)
                    };
                    closeTimer.Tick += (s, args) =>
                    {
                        closeTimer.Stop();
                        Dispatcher.BeginInvoke(DispatcherPriority.Normal, new Action(() =>
                        {
                            try
                            {
                                Close();
                            }
                            catch (Exception ex)
                            {
                                MessageBox.Show($"Error closing splash screen: {ex.Message}");
                            }
                        }));
                    };
                    closeTimer.Start();
                    return;
                }

                // Loading text güncelleme
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
