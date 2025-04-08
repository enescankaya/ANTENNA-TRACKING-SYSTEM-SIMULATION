using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using Project.Models;
using Project.Services;
using GMap.NET;
using GMap.NET.MapProviders;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Media.Animation;

namespace Project
{
    /// <summary>
    /// MainWindow.xaml etkileşim mantığı
    /// </summary>
    public partial class MainWindow : Window
    {
        private AntennaState scanningAntenna;
        private AntennaState directionalAntenna;
        private AirplaneState airplane;
        private bool isScanning;
        private readonly SITLConnection sitlConnection;
        private readonly ParticleSwarmOptimizer pso;
        private readonly KalmanFilter signalFilter;
        private readonly KalmanFilter angleFilter;
        private readonly AntennaController antennaController;
        private DispatcherTimer updateTimer;
        private DispatcherTimer hudUpdateTimer;
        private PointLatLng baseStationPosition;
        private bool isDragging = false;
        private Point dragStart;
        private bool isPanelExpanded = true;
        private readonly double PANEL_WIDTH = 300;

        // Görsel elemanlar için
        private Ellipse planeMarker;
        private Line directionLine;
        private Path radarSweep;
        private TranslateTransform sweepPosition;
        private RotateTransform sweepRotation;
        private DateTime lastMapCenterUpdate = DateTime.MinValue;
        private const int MAP_CENTER_INTERVAL = 5000; // 5 seconds in milliseconds

        private RotateTransform attitudeTransform;
        private double currentHeading;
        private double currentAltitude;
        private double currentSpeed;
        private double currentBattery = 100;
        private double currentThrottle;

        public MainWindow()
        {
            InitializeComponent();

            // Initialize services
            sitlConnection = new SITLConnection();
            antennaController = new AntennaController();
            signalFilter = new KalmanFilter();
            angleFilter = new KalmanFilter();
            pso = new ParticleSwarmOptimizer(); // PSO'yu başlat

            InitializeMap();
            InitializeComponents();
            SetupEventHandlers();
            InitializeVisuals();
            InitializeTimer();
            InitializeHUD();
        }

        private void InitializeComponents()
        {
            // Default IP ve Port değerleri
            IpAddressTextBox.Text = "127.0.0.1";
            PortTextBox.Text = "5762";

            scanningAntenna = new AntennaState();
            directionalAntenna = new AntennaState();
            airplane = new AirplaneState();

            // Initialize UI elements
            UpdateAntennaDisplay();
        }

        private void SetupEventHandlers()
        {
            ConnectButton.Click += ConnectButton_Click;
            StartButton.Click += StartButton_Click;
            ResetButton.Click += ResetButton_Click;
            sitlConnection.OnPositionUpdate += SitlConnection_OnPositionUpdate;
            sitlConnection.OnConnectionStatusChanged += SitlConnection_OnConnectionStatusChanged;
            MapControl.MouseDoubleClick += MapControl_MouseDoubleClick;
            MapControl.MouseLeftButtonDown += MapControl_MouseLeftButtonDown;
            MapControl.MouseLeftButtonUp += MapControl_MouseLeftButtonUp;
            MapControl.MouseMove += MapControl_MouseMove;
            MapControl.MouseWheel += MapControl_MouseWheel;
            DisconnectButton.Click += DisconnectButton_Click;
            TogglePanelButton.Click += TogglePanelButton_Click;
        }

        private void InitializeTimer()
        {
            updateTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100)
            };
            updateTimer.Tick += UpdateTimer_Tick;

            // HUD için timer güncellemesi - interval 100ms yapıldı
            hudUpdateTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100) // Daha sık güncelleme
            };
            hudUpdateTimer.Tick += HudUpdateTimer_Tick;
        }

        private void InitializeVisuals()
        {
            // Radar sweep geometrisi
            var geometry = new PathGeometry();
            var figure = new PathFigure();
            figure.StartPoint = new Point(0, 0);

            // Sweep için yay segmenti
            var arcSegment = new ArcSegment(
                new Point(150, 0),
                new Size(150, 150),
                45, // Sweep angle
                false, // IsLargeArc
                SweepDirection.Clockwise,
                true); // IsStroked

            figure.Segments.Add(new LineSegment(new Point(150, 0), true));
            figure.Segments.Add(arcSegment);
            geometry.Figures.Add(figure);

            if (RadarSweep != null)
            {
                RadarSweep.Data = geometry;

                // Transform group oluştur
                var transformGroup = new TransformGroup();
                sweepRotation = new RotateTransform();
                sweepPosition = new TranslateTransform();

                transformGroup.Children.Add(sweepRotation);
                transformGroup.Children.Add(sweepPosition);

                RadarSweep.RenderTransform = transformGroup;
            }

            // Uçak marker'ı
            planeMarker = new Ellipse
            {
                Width = 10,
                Height = 10,
                Fill = Brushes.Red
            };
            MapCanvas.Children.Add(planeMarker);

            // Yön çizgisi
            directionLine = new Line
            {
                Stroke = Brushes.Green,
                StrokeThickness = 2
            };
            MapCanvas.Children.Add(directionLine);
        }

        private void UpdateMap()
        {
            if (airplane == null || planeMarker == null || directionLine == null || MapCanvas == null)
                return;

            try
            {
                // Only center map every 5 seconds and if scanning
                bool shouldCenterMap = (DateTime.Now - lastMapCenterUpdate).TotalMilliseconds >= MAP_CENTER_INTERVAL
                                      && isScanning;

                if (shouldCenterMap)
                {
                    // Keşan Havalimanı merkezli görünüm
                    MapControl.Position = baseStationPosition;
                    lastMapCenterUpdate = DateTime.Now;
                }

                // Update visual elements
                GPoint gpoint = MapControl.FromLatLngToLocal(new PointLatLng(airplane.Latitude, airplane.Longitude));
                System.Windows.Point screenPoint = new System.Windows.Point(gpoint.X, gpoint.Y);

                // Update airplane marker position
                Canvas.SetLeft(planeMarker, screenPoint.X - planeMarker.Width / 2);
                Canvas.SetTop(planeMarker, screenPoint.Y - planeMarker.Height / 2);

                // Update direction line
                double length = 50;
                double angleRad = airplane.Heading * Math.PI / 180;
                directionLine.X1 = screenPoint.X;
                directionLine.Y1 = screenPoint.Y;
                directionLine.X2 = screenPoint.X + length * Math.Cos(angleRad);
                directionLine.Y2 = screenPoint.X + length * Math.Sin(angleRad);

                if (sweepPosition != null)
                {
                    sweepPosition.X = screenPoint.X;
                    sweepPosition.Y = screenPoint.Y;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"UpdateMap error: {ex.Message}");
            }
        }

        private async void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                string ip = IpAddressTextBox.Text;
                if (!int.TryParse(PortTextBox.Text, out int port))
                {
                    MessageBox.Show("Please enter a valid port number");
                    return;
                }

                await sitlConnection.Connect(ip, port);
                ConnectionStatus.Text = "Connected";
                ConnectionIndicator.Fill = Brushes.Green;
                ConnectButton.IsEnabled = false;
                DisconnectButton.IsEnabled = true;
                IpAddressTextBox.IsEnabled = false;
                PortTextBox.IsEnabled = false;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Connection failed: {ex.Message}");
                ConnectionIndicator.Fill = Brushes.Red;
                ConnectionStatus.Text = "Connection Failed";
            }
        }

        private void StartButton_Click(object sender, RoutedEventArgs e)
        {
            if (!sitlConnection.IsConnected)
            {
                MessageBox.Show("Please connect to vehicle first!");
                return;
            }

            isScanning = !isScanning;
            StartButton.Content = isScanning ? "Stop Scanning" : "Start Scanning";

            if (isScanning)
            {
                MapControl.Position = baseStationPosition;
                MapControl.Zoom = 16;
                updateTimer.Start();
                hudUpdateTimer.Start();
                SystemStatus.Text = "System Active";
            }
            else
            {
                updateTimer.Stop();
                hudUpdateTimer.Stop();
                SystemStatus.Text = "System Stopped";
            }
        }

        private void ResetButton_Click(object sender, RoutedEventArgs e)
        {
            MapCanvas.Children.Clear();
            InitializeVisuals();
            antennaController.Reset();
            scanningAntenna = new AntennaState();
            directionalAntenna = new AntennaState();
            UpdateAntennaDisplay();
        }

        private void SitlConnection_OnPositionUpdate(object sender, AirplaneState e)
        {
            try
            {
                airplane = e;
                currentHeading = airplane.Heading;
                currentAltitude = airplane.Altitude;
                currentSpeed = airplane.GroundSpeed;
                currentBattery = airplane.Battery;
                currentThrottle = airplane.Throttle;

                // Her MAVLink mesajıyla HUD'u güncelle
                if (isScanning)
                {
                    Dispatcher.Invoke(() =>
                    {
                        UpdateAntennaDisplay();
                        UpdateMap();
                        UpdateHUD(currentHeading, currentAltitude, currentSpeed, currentBattery, currentThrottle);
                    });
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Position update error: {ex.Message}");
            }
        }

        private void SitlConnection_OnConnectionStatusChanged(object sender, string status)
        {
            Dispatcher.Invoke(() => ConnectionStatus.Text = status);
        }

        private async void UpdateTimer_Tick(object sender, EventArgs e)
        {
            if (!isScanning || airplane == null) return;

            await Task.Run(() =>
            {
                antennaController.UpdateScanningAntenna(scanningAntenna, airplane);
                antennaController.UpdateDirectionalAntenna(directionalAntenna, airplane);
            });

            // UI güncellemeleri
            UpdateAntennaDisplay();
            UpdateMap();
            _ = UpdateRadarPosition(); // Fire and forget async call
        }

        private void HudUpdateTimer_Tick(object sender, EventArgs e)
        {
            if (airplane != null && sitlConnection.IsConnected)
            {
                try
                {
                    Dispatcher.Invoke(() =>
                    {
                        UpdateHUD(
                            currentHeading,
                            currentAltitude,
                            currentSpeed,
                            currentBattery,
                            currentThrottle
                        );
                    });
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"HUD timer error: {ex.Message}");
                }
            }
        }

        private void UpdateAntennaPositions()
        {
            // Get next scanning position from PSO
            double nextAngle = pso.GetNextAngle(scanningAntenna.SignalStrength);
            scanningAntenna.HorizontalAngle = angleFilter.Update(nextAngle);

            // Update signal strength with Kalman filter
            double rawSignal = CalculateSignalStrength(scanningAntenna.HorizontalAngle);
            scanningAntenna.SignalStrength = signalFilter.Update(rawSignal);
        }

        private double CalculateSignalStrength(double angle)
        {
            // Simulate signal strength based on antenna angle and airplane position
            // This is a simplified calculation - actual implementation would be more complex
            double angleDiff = Math.Abs(angle - airplane.Heading);
            return Math.Max(0, 100 - (angleDiff / 360.0 * 100));
        }

        private void UpdateAntennaDisplay()
        {
            Dispatcher.Invoke(() =>
            {
                try
                {
                    // Antenna angles
                    if (scanningAntenna != null)
                    {
                        ScanHAngle.Text = $"{scanningAntenna.HorizontalAngle:F1}°";
                        ScanVAngle.Text = $"{scanningAntenna.VerticalAngle:F1}°";
                        SignalStrength.Text = $"{scanningAntenna.SignalStrength:F1}%";
                        ScanSignalBar.Value = scanningAntenna.SignalStrength;
                    }

                    if (directionalAntenna != null)
                    {
                        DirHAngle.Text = $"{directionalAntenna.HorizontalAngle:F1}°";
                        DirVAngle.Text = $"{directionalAntenna.VerticalAngle:F1}°";
                        DirSignalBar.Value = directionalAntenna.SignalStrength;
                        DirSignalText.Text = $"{directionalAntenna.SignalStrength:F1}%";
                    }

                    // Update radar visualization
                    UpdateRadarDisplay();

                    // Update status message
                    StatusMessage.Text = isScanning ?
                        $"Scanning: {scanningAntenna?.SignalStrength:F1}% Signal" :
                        "Ready";
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"UpdateAntennaDisplay error: {ex.Message}");
                }
            });
        }

        private void InitializeMap()
        {
            try
            {
                // Cache ve proxy ayarları
                GMaps.Instance.Mode = AccessMode.ServerAndCache;
                GMap.NET.MapProviders.GMapProvider.WebProxy = System.Net.WebRequest.GetSystemWebProxy();
                GMap.NET.MapProviders.GMapProvider.WebProxy.Credentials = System.Net.CredentialCache.DefaultCredentials;

                // Google Satellite provider'ı kullan
                MapControl.MapProvider = GMap.NET.MapProviders.GoogleSatelliteMapProvider.Instance;
                GMaps.Instance.Mode = AccessMode.ServerAndCache;

                // Keşan Havalimanı koordinatları
                baseStationPosition = new PointLatLng(40.737222, 26.571667);
                MapControl.Position = baseStationPosition;

                // Uydu görüntüsü için daha yakın zoom
                MapControl.MinZoom = 2;
                MapControl.MaxZoom = 20; // Satellite için max zoom artırıldı
                MapControl.Zoom = 16;    // Başlangıç zoom seviyesi artırıldı

                // Harita kontrolleri
                MapControl.ShowCenter = false;
                MapControl.CanDragMap = true;
                MapControl.DragButton = MouseButton.Left;
                MapControl.MouseWheelZoomEnabled = true;
                MapControl.MouseWheelZoomType = MouseWheelZoomType.MousePositionAndCenter;

                // Performans ayarları
                MapControl.RetryLoadTile = 0;
                MapControl.ShowTileGridLines = false;

                // Önemli: Panel'in üstte kalmasını sağla
                Panel.SetZIndex(MapCanvas, 1);

                UpdateRadarPosition();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Harita başlatılamadı: {ex.Message}");
            }
        }

        private async Task UpdateRadarPosition()
        {
            if (MapControl == null || airplane == null) return;

            try
            {
                // Uçak pozisyonunu al
                GPoint planePoint = MapControl.FromLatLngToLocal(new PointLatLng(airplane.Latitude, airplane.Longitude));

                // Radar elemanlarını uçak pozisyonuna göre konumlandır
                if (RadarBackground != null)
                {
                    Canvas.SetLeft(RadarBackground, planePoint.X - RadarBackground.Width / 2);
                    Canvas.SetTop(RadarBackground, planePoint.Y - RadarBackground.Height / 2);
                }

                if (RadarGrid != null)
                {
                    Canvas.SetLeft(RadarGrid, planePoint.X - 100); // Grid merkezi için
                    Canvas.SetTop(RadarGrid, planePoint.Y - 100);
                }

                if (RadarSweep != null)
                {
                    Canvas.SetLeft(RadarSweep, planePoint.X - 50); // Sweep merkezi için
                    Canvas.SetTop(RadarSweep, planePoint.Y - 50);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Radar position update error: {ex.Message}");
            }
        }

        private async void UpdateRadarRotation(double angle)
        {
            await Dispatcher.InvokeAsync(() =>
            {
                if (sweepRotation != null)
                {
                    sweepRotation.Angle = angle;
                }
            });
        }

        private void MapControl_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            Point mousePosition = e.GetPosition(MapControl);
            baseStationPosition = MapControl.FromLocalToLatLng((int)mousePosition.X, (int)mousePosition.Y);
            UpdateRadarPosition();
        }

        private async void MapControl_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left && e.ButtonState == MouseButtonState.Pressed)
            {
                isDragging = true;
                dragStart = e.GetPosition(MapControl);
                try
                {
                    await Dispatcher.InvokeAsync(() => MapControl.CaptureMouse());
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"Mouse capture error: {ex.Message}");
                    isDragging = false;
                }
            }
        }

        private void MapControl_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
            {
                isDragging = false;
                MapControl.ReleaseMouseCapture();
            }
        }

        private void MapControl_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isDragging) return;

            try
            {
                Point current = e.GetPosition(MapControl);

                double offsetX = current.X - dragStart.X;
                double offsetY = current.Y - dragStart.Y;

                dragStart = current;

                // Daha hassas sürükleme hesaplaması
                double scale = 1 << (int)MapControl.Zoom;
                PointLatLng currentPosition = MapControl.Position;

                double newLat = currentPosition.Lat - (offsetY / scale);
                double newLng = currentPosition.Lng + (offsetX / scale);

                MapControl.Position = new PointLatLng(newLat, newLng);

                // Radar pozisyonunu güncelle
                UpdateRadarPosition();

                e.Handled = true;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Mouse move error: {ex.Message}");
            }
        }

        private void MapControl_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            try
            {
                if (e.Delta > 0)
                {
                    if (MapControl.Zoom < MapControl.MaxZoom)
                        MapControl.Zoom += 1;
                }
                else
                {
                    if (MapControl.Zoom > MapControl.MinZoom)
                        MapControl.Zoom -= 1;
                }

                e.Handled = true;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Mouse wheel error: {ex.Message}");
            }
        }

        // OnRenderSizeChanged yerine SizeChanged event handler kullanacağız
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            InitializeVisuals();
            // Window boyut değişikliklerini dinle
            SizeChanged += MainWindow_SizeChanged;
        }

        private void MainWindow_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            UpdateRadarPosition();
        }

        private void UpdateRadarDisplay()
        {
            if (RadarSweep != null && scanningAntenna != null)
            {
                // Radar sweep rotation güncelleme
                if (sweepRotation != null)
                {
                    sweepRotation.Angle = scanningAntenna.HorizontalAngle;

                    // Sinyal gücüne göre radar renk ve opaklık ayarı
                    byte alpha = (byte)(scanningAntenna.SignalStrength * 1.5);
                    var gradientBrush = new RadialGradientBrush();
                    gradientBrush.GradientStops.Add(new GradientStop(Color.FromArgb(alpha, 0, 255, 0), 0));
                    gradientBrush.GradientStops.Add(new GradientStop(Color.FromArgb(0, 0, 255, 0), 1));
                    RadarSweep.Fill = gradientBrush;
                }
            }

            // Signal bar güncelleme
            if (ScanSignalBar != null)
            {
                ScanSignalBar.Value = scanningAntenna?.SignalStrength ?? 0;
            }
        }

        private void NumberValidationTextBox(object sender, TextCompositionEventArgs e)
        {
            e.Handled = !IsTextAllowed(e.Text);
        }

        private static bool IsTextAllowed(string text)
        {
            return text.All(char.IsDigit) || text == ".";
        }

        public void DisconnectAndReset()
        {
            ConnectButton.IsEnabled = true;
            DisconnectButton.IsEnabled = false;
            IpAddressTextBox.IsEnabled = true;
            PortTextBox.IsEnabled = true;
            ConnectionIndicator.Fill = Brushes.Red;
            ConnectionStatus.Text = "Disconnected";

            // Stop scanning if active
            if (isScanning)
            {
                isScanning = false;
                updateTimer.Stop();
                StartButton.Content = "Start Scanning";
            }
        }

        private void DisconnectButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                sitlConnection.Disconnect();
                DisconnectAndReset();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Disconnect error: {ex.Message}");
            }
        }

        private void TogglePanelButton_Click(object sender, RoutedEventArgs e)
        {
            isPanelExpanded = !isPanelExpanded;

            var animation = new DoubleAnimation
            {
                To = isPanelExpanded ? PANEL_WIDTH : 0,
                Duration = TimeSpan.FromMilliseconds(250),
                EasingFunction = new QuadraticEase()
            };

            animation.Completed += (s, _) =>
            {
                if (!isPanelExpanded)
                    ControlPanel.Visibility = Visibility.Collapsed;
            };

            if (isPanelExpanded)
                ControlPanel.Visibility = Visibility.Visible;

            ControlPanel.BeginAnimation(WidthProperty, animation);
            rightPanelColumn.Width = new GridLength(isPanelExpanded ? PANEL_WIDTH : 0);
            TogglePanelButton.Content = isPanelExpanded ? "⟨" : "⟩";
        }

        private void InitializeHUD()
        {
            attitudeTransform = new RotateTransform();
            if (ArtificialHorizon != null)
            {
                ArtificialHorizon.RenderTransform = attitudeTransform;
            }
            UpdateHUD(0, 100, 0, 100, 50); // Initial values
        }

        private void UpdateHUD(double heading, double altitude, double speed, double battery, double throttle)
        {
            if (!sitlConnection.IsConnected) return;

            Dispatcher.Invoke(() =>
            {
                try
                {
                    // Heading formatı (000° - 359°)
                    HeadingText.Text = $"HDG: {heading:000}°";

                    // Altitude formatı (±00000)
                    AltitudeText.Text = $"ALT: {altitude:F0}m";

                    // Ground Speed (gerçek değer)
                    SpeedText.Text = $"GS: {speed:F1}m/s";

                    // Battery formatı (00.0V)
                    BatteryText.Text = $"BAT: {battery:F1}V";

                    // Throttle formatı (000%)
                    ThrottleText.Text = $"THR: {throttle:F0}%";

                    // Progress barları güncelle
                    BatteryIndicator.Value = battery;
                    ThrottleIndicator.Value = throttle;

                    // Artificial horizon güncelle
                    if (airplane != null)
                    {
                        UpdateArtificialHorizon(airplane.Pitch, airplane.Roll);
                    }

                    // Compass rose güncelle
                    UpdateCompassRose(heading);

                    // Speed ve Altitude tape'leri güncelle
                    UpdateSpeedTape(speed);
                    UpdateAltitudeTape(altitude);
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"HUD update error: {ex.Message}");
                }
            });
        }

        private void UpdateArtificialHorizon(double pitch, double roll)
        {
            var transform = new TransformGroup();
            transform.Children.Add(new RotateTransform(roll, 75, 75));

            // Pitch için y-ekseni kaydırma
            transform.Children.Add(new TranslateTransform(0, pitch));

            ArtificialHorizon.RenderTransform = transform;
        }

        private void UpdateCompassRose(double heading)
        {
            CompassRose.Children.Clear();

            for (int i = 0; i < 360; i += 30)
            {
                double adjustedAngle = (i - heading + 360) % 360;
                double x = 140 + 130 * Math.Sin(adjustedAngle * Math.PI / 180);
                if (x >= 0 && x <= 280)
                {
                    TextBlock tickLabel = new TextBlock
                    {
                        Text = i.ToString(),
                        Foreground = Brushes.White,
                        FontSize = 10
                    };
                    Canvas.SetLeft(tickLabel, x - 10);
                    Canvas.SetTop(tickLabel, 10);
                    CompassRose.Children.Add(tickLabel);
                }
            }
        }

        private void UpdateSpeedTape(double speed)
        {
            SpeedTape.Children.Clear();
            int baseSpeed = (int)speed;

            // Anlık hız göstergesi (yeşil şeffaf kutu)
            var currentSpeedIndicator = new Border
            {
                Width = 40,
                Height = 20,
                Background = new SolidColorBrush(Color.FromArgb(64, 0, 255, 0)),
                Child = new TextBlock
                {
                    Text = $"{speed:F1}",
                    Foreground = Brushes.White,
                    FontSize = 12,
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center
                }
            };
            Canvas.SetLeft(currentSpeedIndicator, 0);
            Canvas.SetTop(currentSpeedIndicator, 90); // Merkeze yakın
            SpeedTape.Children.Add(currentSpeedIndicator);

            // Diğer hız değerleri
            for (int i = -5; i <= 5; i++)
            {
                int tapeSpeed = baseSpeed + i * 5;
                if (tapeSpeed < 0) continue;

                var text = new TextBlock
                {
                    Text = tapeSpeed.ToString(),
                    Foreground = Brushes.White,
                    FontSize = 12
                };
                Canvas.SetLeft(text, 5);
                Canvas.SetTop(text, 100 - i * 20);
                SpeedTape.Children.Add(text);
            }
        }

        private void UpdateAltitudeTape(double altitude)
        {
            AltitudeTape.Children.Clear();
            int baseAlt = (int)altitude;
            for (int i = -5; i <= 5; i++)
            {
                int tapeAlt = baseAlt + i * 10;
                var text = new TextBlock
                {
                    Text = tapeAlt.ToString(),
                    Foreground = Brushes.White,
                    FontSize = 12
                };
                Canvas.SetRight(text, 5);
                Canvas.SetTop(text, 100 - i * 20);
                AltitudeTape.Children.Add(text);
            }
        }
    }
}
