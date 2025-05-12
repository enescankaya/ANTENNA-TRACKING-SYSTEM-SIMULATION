using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Threading;
using Project.Views;

namespace Project
{
    /// <summary>
    /// App.xaml etkileşim mantığı
    /// </summary>
    public partial class App : Application
    {
        private MainWindow mainWindow;

        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);

            try
            {
                // Ana pencereyi şimdiden oluştur ama gösterme
                mainWindow = new MainWindow();

                // Splash screen'i göster
                var splashScreen = new Views.SplashScreen();
                splashScreen.Closed += SplashScreen_Closed; // Kapanma eventi ekle
                splashScreen.ShowDialog();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Application start error: {ex.Message}", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
                Shutdown(-1);
            }
        }

        private void SplashScreen_Closed(object sender, EventArgs e)
        {
            try
            {
                // Splash screen kapandığında ana pencereyi göster
                Dispatcher.BeginInvoke(DispatcherPriority.Normal, new Action(() =>
                {
                    if (mainWindow != null)
                    {
                        mainWindow.Show();
                    }
                    else
                    {
                        MessageBox.Show("Main window initialization failed.", "Error");
                        Shutdown(-1);
                    }
                }));
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error showing main window: {ex.Message}", "Error");
                Shutdown(-1);
            }
        }
    }
}
