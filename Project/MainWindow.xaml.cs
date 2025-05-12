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
using static Project.Services.AntennaController;
using System.Collections.Generic;

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
        private PointLatLng antennaPosition = new PointLatLng(40.7872, 26.6068);
        private bool isDragging = false;
        private Point dragStart;
        private bool isPanelExpanded = true;
        private readonly double PANEL_WIDTH = 300;

        // Add these fields with other state variables
        private double currentScanArea = 360.0;
        private const double MIN_SCAN_AREA = 30.0;

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

        private Button centerMapButton;
        private Line antennaDirectionLine;
        private Ellipse antennaMarker;
        private Ellipse aircraftMarker;

        // Yönlenme anteni güncelleme için zamanlayıcı
        private DateTime lastDirectionalUpdate = DateTime.MinValue;
        private const int DIRECTIONAL_UPDATE_PERIOD_MS = 2000; // 2 saniyede bir yönlenme anteni güncellensin

        // Tarama anteninin bulduğu en iyi sinyal
        private double lastBestScanSignal = 0;

        public MainWindow()
        {
            InitializeComponent();

            // Initialize services
            sitlConnection = new SITLConnection();
            antennaController = new AntennaController();
            signalFilter = new KalmanFilter();
            angleFilter = new KalmanFilter();
            pso = new ParticleSwarmOptimizer();

            InitializeMap();
            InitializeVisuals();
            InitializeComponents();
            SetupEventHandlers();
            InitializeTimer();
            InitializeHUD();

            // Ortalama butonunu en son ve bir kere bağla
            if (centerMapButton != null)
            {
                centerMapButton.Click -= CenterMapButton_Click; // Varsa önceki bağlantıyı kaldır
                centerMapButton.Click += CenterMapButton_Click; // Yeni bağlantı ekle
            }
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
            // Remove centerMapButton.Click registration from here since it's handled in MainWindow constructor
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
                Interval = TimeSpan.FromMilliseconds(500) // 500ms'e çıkarıldı
            };
            updateTimer.Tick += UpdateTimer_Tick;

            hudUpdateTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(500) // 500ms'e çıkarıldı
            };
            hudUpdateTimer.Tick += HudUpdateTimer_Tick;
        }

        private void InitializeVisuals()
        {
            // Radar sweep geometrisi
            var geometry = new PathGeometry();
            var figure = new PathFigure();
            figure.StartPoint = new Point(0, 0);

            // Sweep için yay segmenti - tarama açısını temsil eden daha geniş bir yay
            var arcSegment = new ArcSegment(
                new Point(150, 0),
                new Size(150, 150),
                60, // Daha geniş sweep angle
                false,
                SweepDirection.Clockwise,
                true);

            figure.Segments.Add(new LineSegment(new Point(150, 0), true));
            figure.Segments.Add(arcSegment);
            geometry.Figures.Add(figure);

            // Tarama anteni (yeşil) - scan çizgisi
            Line scanningLine = new Line
            {
                Stroke = new SolidColorBrush(Color.FromArgb(255, 0, 255, 0)), // Tam yeşil ve opak
                StrokeThickness = 5,
                Opacity = 1.0,
                StrokeDashArray = new DoubleCollection { 6, 2 },
                Effect = new System.Windows.Media.Effects.DropShadowEffect
                {
                    Color = Colors.Lime,
                    BlurRadius = 10,
                    ShadowDepth = 0,
                    Opacity = 0.8
                }
            };
            MapCanvas.Children.Add(scanningLine);

            // Yönlenme anteni (mavi)
            Line directionalLine = new Line
            {
                Stroke = new SolidColorBrush(Color.FromArgb(255, 0, 160, 255)),
                StrokeThickness = 3,
                Opacity = 0.8,
                Effect = new System.Windows.Media.Effects.DropShadowEffect
                {
                    Color = Colors.DeepSkyBlue,
                    BlurRadius = 8,
                    ShadowDepth = 0,
                    Opacity = 0.7
                }
            };
            MapCanvas.Children.Add(directionalLine);

            // Anten gösterge açıklamaları - küçültülmüş hali
            var legendBorder = new Border
            {
                Background = new SolidColorBrush(Color.FromArgb(128, 0, 0, 0)),
                Margin = new Thickness(5),
                Padding = new Thickness(3),
                CornerRadius = new CornerRadius(3)
            };

            var legend = new StackPanel
            {
                Orientation = Orientation.Horizontal
            };

            legend.Children.Add(new Rectangle { Width = 15, Height = 2, Fill = Brushes.Green, Margin = new Thickness(2) });
            legend.Children.Add(new TextBlock { Text = "Scan", Foreground = Brushes.White, Margin = new Thickness(2), FontSize = 10 });
            legend.Children.Add(new Rectangle { Width = 15, Height = 2, Fill = Brushes.Blue, Margin = new Thickness(8, 2, 2, 2) });
            legend.Children.Add(new TextBlock { Text = "Track", Foreground = Brushes.White, Margin = new Thickness(2), FontSize = 10 });

            legendBorder.Child = legend;
            Canvas.SetLeft(legendBorder, 5);
            Canvas.SetBottom(legendBorder, 5);
            MapCanvas.Children.Add(legendBorder);

            // Anten marker'ı
            antennaMarker = new Ellipse
            {
                Width = 10,
                Height = 10,
                Fill = Brushes.Yellow,
                Stroke = Brushes.Orange,
                StrokeThickness = 2
            };

            // Uçak marker'ı
            aircraftMarker = new Ellipse
            {
                Width = 12,
                Height = 12,
                Fill = Brushes.Red,
                Stroke = Brushes.White,
                StrokeThickness = 2
            };

            // Anten yön çizgisi
            antennaDirectionLine = new Line
            {
                Stroke = new SolidColorBrush(Color.FromArgb(180, 255, 165, 0)),
                StrokeThickness = 2,
                StrokeDashArray = new DoubleCollection { 4, 2 }
            };

            RadarElementsCanvas.Children.Add(antennaDirectionLine);
            RadarElementsCanvas.Children.Add(antennaMarker);
            RadarElementsCanvas.Children.Add(aircraftMarker);
        }

        private void CenterMapButton_Click(object sender, RoutedEventArgs e)
        {
            if (airplane == null) return;

            try
            {
                // Anten ve uçak arasındaki orta noktayı hesapla
                double centerLat = (antennaPosition.Lat + airplane.Latitude) / 2;
                double centerLng = (antennaPosition.Lng + airplane.Longitude) / 2;

                // Mesafeyi hesapla
                double latDiff = Math.Abs(antennaPosition.Lat - airplane.Latitude);
                double lngDiff = Math.Abs(antennaPosition.Lng - airplane.Longitude);
                double maxDiff = Math.Max(latDiff, lngDiff) * 2;

                // Zoom seviyesini ayarla
                int zoomLevel = 15;
                if (maxDiff > 0.01) zoomLevel = 14;
                if (maxDiff > 0.02) zoomLevel = 13;
                if (maxDiff > 0.05) zoomLevel = 12;
                if (maxDiff > 0.1) zoomLevel = 11;

                // Merkez ve zoom'u ayarla
                MapControl.Position = new PointLatLng(centerLat, centerLng);
                MapControl.Zoom = zoomLevel;

                // Radar pozisyonunu güncelle
                UpdateRadarPosition();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Center map error: {ex.Message}");
            }
        }

        private void AnimateCanvasElementPosition(UIElement element, double toLeft, double toTop, double duration = 400)
        {
            if (element == null) return;

            var fromLeft = Canvas.GetLeft(element);
            var fromTop = Canvas.GetTop(element);

            // Create left animation
            var leftAnimation = new DoubleAnimation
            {
                From = double.IsNaN(fromLeft) ? toLeft : fromLeft,
                To = toLeft,
                Duration = TimeSpan.FromMilliseconds(duration),
                EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
            };

            // Create top animation
            var topAnimation = new DoubleAnimation
            {
                From = double.IsNaN(fromTop) ? toTop : fromTop,
                To = toTop,
                Duration = TimeSpan.FromMilliseconds(duration),
                EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
            };

            // Set final values for layout
            Canvas.SetLeft(element, toLeft);
            Canvas.SetTop(element, toTop);

            // Start animations
            element.BeginAnimation(Canvas.LeftProperty, leftAnimation);
            element.BeginAnimation(Canvas.TopProperty, topAnimation);
        }

        private void AnimateLinePosition(Line line, double x1, double y1, double x2, double y2, double duration = 400)
        {
            if (line == null) return;

            // Create animations for each coordinate
            var x1Animation = new DoubleAnimation(x1, TimeSpan.FromMilliseconds(duration))
            {
                EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
            };
            var y1Animation = new DoubleAnimation(y1, TimeSpan.FromMilliseconds(duration))
            {
                EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
            };
            var x2Animation = new DoubleAnimation(x2, TimeSpan.FromMilliseconds(duration))
            {
                EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
            };
            var y2Animation = new DoubleAnimation(y2, TimeSpan.FromMilliseconds(duration))
            {
                EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
            };

            // Start animations
            line.BeginAnimation(Line.X1Property, x1Animation);
            line.BeginAnimation(Line.Y1Property, y1Animation);
            line.BeginAnimation(Line.X2Property, x2Animation);
            line.BeginAnimation(Line.Y2Property, y2Animation);
        }

        private void UpdateMap()
        {
            try
            {
                GPoint antennaPoint = MapControl.FromLatLngToLocal(antennaPosition);

                // Animate antenna marker with smooth easing
                AnimateCanvasElementPosition(antennaMarker,
                    antennaPoint.X - antennaMarker.Width / 2,
                    antennaPoint.Y - antennaMarker.Height / 2);

                // Animate aircraft marker if available
                if (airplane != null)
                {
                    GPoint planePoint = MapControl.FromLatLngToLocal(
                        new PointLatLng(airplane.Latitude, airplane.Longitude));

                    AnimateCanvasElementPosition(aircraftMarker,
                        planePoint.X - aircraftMarker.Width / 2,
                        planePoint.Y - aircraftMarker.Height / 2);

                    // Animate antenna direction line
                    AnimateLinePosition(antennaDirectionLine,
                        antennaPoint.X, antennaPoint.Y,
                        planePoint.X, planePoint.Y);
                }

                // Update radar elements with animations
                if (RadarBackground != null)
                {
                    AnimateCanvasElementPosition(RadarBackground,
                        antennaPoint.X - RadarBackground.Width / 2,
                        antennaPoint.Y - RadarBackground.Height / 2);
                }

                // Update scanning and directional antenna lines with smooth animations
                UpdateAntennaLines(antennaPoint);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"UpdateMap error: {ex.Message}");
            }
        }

        private void UpdateAntennaLines(GPoint antennaPoint)
        {
            // Scan çizgisini her zaman bulmak için: Stroke rengini kontrol etme, MapCanvas.Children'daki ilk Line'ı scan olarak kullan
            Line scanningLine = MapCanvas.Children.OfType<Line>().FirstOrDefault();
            if (scanningLine != null && scanningAntenna != null)
            {
                double scanAngleRad = (scanningAntenna.HorizontalAngle - 90) * Math.PI / 180;
                double scanV = scanningAntenna.VerticalAngle;
                double length = 120 + 40 * (scanV / 90.0);

                // Özellikleri tekrar uygula (görünürlük için)
                scanningLine.Stroke = new SolidColorBrush(Color.FromArgb(255, 0, 255, 0));
                scanningLine.StrokeThickness = 5;
                scanningLine.Opacity = 1.0;
                scanningLine.StrokeDashArray = new DoubleCollection { 6, 2 };
                scanningLine.Effect = new System.Windows.Media.Effects.DropShadowEffect
                {
                    Color = Colors.Lime,
                    BlurRadius = 10,
                    ShadowDepth = 0,
                    Opacity = 0.8
                };

                AnimateLinePosition(scanningLine,
                    antennaPoint.X, antennaPoint.Y,
                    antennaPoint.X + length * Math.Cos(scanAngleRad),
                    antennaPoint.Y + length * Math.Sin(scanAngleRad));
            }

            // Directional çizgiyi MapCanvas.Children'daki ikinci Line olarak al
            Line directionalLine = MapCanvas.Children.OfType<Line>().Skip(1).FirstOrDefault();
            if (directionalLine != null && directionalAntenna != null)
            {
                double dirAngleRad = (directionalAntenna.HorizontalAngle - 90) * Math.PI / 180;
                double dirV = directionalAntenna.VerticalAngle;
                double length = 160 + 40 * (dirV / 90.0);

                directionalLine.Stroke = new SolidColorBrush(Color.FromArgb(255, 0, 160, 255));
                directionalLine.StrokeThickness = 3;
                directionalLine.Opacity = 0.8;
                directionalLine.StrokeDashArray = null;
                directionalLine.Effect = new System.Windows.Media.Effects.DropShadowEffect
                {
                    Color = Colors.DeepSkyBlue,
                    BlurRadius = 8,
                    ShadowDepth = 0,
                    Opacity = 0.7
                };

                AnimateLinePosition(directionalLine,
                    antennaPoint.X, antennaPoint.Y,
                    antennaPoint.X + length * Math.Cos(dirAngleRad),
                    antennaPoint.Y + length * Math.Sin(dirAngleRad));
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
            // Anten kontrolcüsünü sıfırla
            antennaController.Reset();
            scanningAntenna = new AntennaState();
            directionalAntenna = new AntennaState();

            // UI göstergelerini sıfırla
            UpdateAntennaDisplay();
            UpdateRadarPosition();

            // HUD'u sıfırla
            UpdateHUD(0, 0, 0, 100, 0);

            // Çizgileri orijinal pozisyonlarına getir
            if (antennaDirectionLine != null)
            {
                double centerX = RadarElementsCanvas.ActualWidth / 2;
                double centerY = RadarElementsCanvas.ActualHeight / 2;
                antennaDirectionLine.X1 = centerX;
                antennaDirectionLine.Y1 = centerY;
                antennaDirectionLine.X2 = centerX;
                antennaDirectionLine.Y2 = centerY - 100; // Varsayılan uzunluk
            }

            // Map üzerindeki çizgileri sıfırla
            var scanningLine = MapCanvas.Children.OfType<Line>().FirstOrDefault(l => l.Stroke == Brushes.Green);
            var directionalLine = MapCanvas.Children.OfType<Line>().FirstOrDefault(l => l.Stroke == Brushes.Blue);
            if (scanningLine != null)
            {
                scanningLine.X1 = MapCanvas.ActualWidth / 2;
                scanningLine.Y1 = MapCanvas.ActualHeight / 2;
                scanningLine.X2 = MapCanvas.ActualWidth / 2;
                scanningLine.Y2 = MapCanvas.ActualHeight / 2 - 100;
            }
            if (directionalLine != null)
            {
                directionalLine.X1 = MapCanvas.ActualWidth / 2;
                directionalLine.Y1 = MapCanvas.ActualHeight / 2;
                directionalLine.X2 = MapCanvas.ActualWidth / 2;
                directionalLine.Y2 = MapCanvas.ActualHeight / 2 - 160;
            }

            // Uçak marker'ını ve anten marker'ını merkeze getir
            if (aircraftMarker != null)
            {
                Canvas.SetLeft(aircraftMarker, MapCanvas.ActualWidth / 2 - aircraftMarker.Width / 2);
                Canvas.SetTop(aircraftMarker, MapCanvas.ActualHeight / 2 - aircraftMarker.Height / 2);
            }
            if (antennaMarker != null)
            {
                Canvas.SetLeft(antennaMarker, MapCanvas.ActualWidth / 2 - antennaMarker.Width / 2);
                Canvas.SetTop(antennaMarker, MapCanvas.ActualHeight / 2 - antennaMarker.Height / 2);
            }

            // RadarGrid'i görünür yap ve merkeze konumlandır
            if (RadarGrid != null)
            {
                RadarGrid.Visibility = Visibility.Visible;
                Canvas.SetLeft(RadarGrid, MapCanvas.ActualWidth / 2 - 100);
                Canvas.SetTop(RadarGrid, MapCanvas.ActualHeight / 2 - 100);
            }

            // Radar Background'u görünür yap ve merkeze konumlandır
            if (RadarBackground != null)
            {
                RadarBackground.Visibility = Visibility.Visible;
                Canvas.SetLeft(RadarBackground, MapCanvas.ActualWidth / 2 - RadarBackground.Width / 2);
                Canvas.SetTop(RadarBackground, MapCanvas.ActualHeight / 2 - RadarBackground.Height / 2);
            }

            // Radar ve kontrol panelini görünür yap
            if (ControlPanel != null) ControlPanel.Visibility = Visibility.Visible;

            // Sistem durumunu güncelle
            SystemStatus.Text = "System Reset";
            StatusMessage.Text = "Ready";

            // Map'i güncelle
            UpdateMap();
        }

        private async void SitlConnection_OnPositionUpdate(object sender, AirplaneState e)
        {
            if (e == null) return;

            try
            {
                airplane = e;
                currentHeading = e.Heading;
                currentAltitude = e.Altitude;
                currentSpeed = e.GroundSpeed;
                currentBattery = e.Battery;
                currentThrottle = e.Throttle; // Fix: Properly assign throttle value

                // Calculate angles from antenna to aircraft
                double targetAngleH = CalculateHorizontalAngle(
                    antennaPosition.Lat,
                    antennaPosition.Lng,
                    e.Latitude,
                    e.Longitude
                );

                double targetAngleV = CalculateVerticalAngle(
                    antennaPosition.Lat,
                    antennaPosition.Lng,
                    0,
                    e.Latitude,
                    e.Longitude,
                    e.Altitude
                );

                await Dispatcher.InvokeAsync(() =>
                {
                    try
                    {
                        // Update aircraft position angles relative to antenna
                        AircraftHAngle.Text = $"{targetAngleH:F1}°";
                        AircraftVAngle.Text = $"{targetAngleV:F1}°";

                        // ...rest of the UI updates...
                    }
                    catch (Exception ex)
                    {
                        System.Diagnostics.Debug.WriteLine($"UI update error: {ex.Message}");
                    }
                }, System.Windows.Threading.DispatcherPriority.Render);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Position update error: {ex.Message}");
            }
        }

        private double CalculateHorizontalAngle(double lat1, double lon1, double lat2, double lon2)
        {
            double dLon = (lon2 - lon1) * Math.PI / 180.0;
            double y = Math.Sin(dLon) * Math.Cos(lat2 * Math.PI / 180.0);
            double x = Math.Cos(lat1 * Math.PI / 180.0) * Math.Sin(lat2 * Math.PI / 180.0) -
                      Math.Sin(lat1 * Math.PI / 180.0) * Math.Cos(lat2 * Math.PI / 180.0) * Math.Cos(dLon);
            double angle = Math.Atan2(y, x) * 180.0 / Math.PI;
            return (angle + 360) % 360;
        }

        private double CalculateVerticalAngle(double lat1, double lon1, double alt1,
                                            double lat2, double lon2, double alt2)
        {
            double R = 6371000; // Earth radius in meters
            double dLat = (lat2 - lat1) * Math.PI / 180.0;
            double dLon = (lon2 - lon1) * Math.PI / 180.0;

            // Calculate great circle distance
            double a = Math.Sin(dLat / 2) * Math.Sin(dLat / 2) +
                       Math.Cos(lat1 * Math.PI / 180.0) * Math.Cos(lat2 * Math.PI / 180.0) *
                       Math.Sin(dLon / 2) * Math.Sin(dLon / 2);
            double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
            double distance = R * c;

            // Calculate vertical angle
            double heightDiff = alt2 - alt1;
            return Math.Atan2(heightDiff, distance) * 180.0 / Math.PI;
        }

        private void SitlConnection_OnConnectionStatusChanged(object sender, string status)
        {
            Dispatcher.Invoke(() => ConnectionStatus.Text = status);
        }

        private async void UpdateTimer_Tick(object sender, EventArgs e)
        {
            if (!isScanning || airplane == null) return;

            try
            {
                // Update scanning antenna position
                scanningAntenna.Latitude = antennaPosition.Lat;
                scanningAntenna.Longitude = antennaPosition.Lng;
                scanningAntenna.Altitude = 0;

                // Update directional antenna position
                directionalAntenna.Latitude = antennaPosition.Lat;
                directionalAntenna.Longitude = antennaPosition.Lng;
                directionalAntenna.Altitude = 0;

                // Update scanning antenna using PSO with periodic reset
                double currentRssi = antennaController.UpdateScanningAntenna(scanningAntenna, airplane);

                // If we got a good RSSI, update directional antenna
                if (currentRssi > -85) // -85 dBm threshold for usable signal
                {
                    antennaController.UpdateDirectionalAntenna(directionalAntenna, scanningAntenna, airplane);
                }

                // Her durumda directional anteni güncelle
                antennaController.UpdateDirectionalAntenna(directionalAntenna, scanningAntenna, airplane);

                // Refresh UI elements
                await Dispatcher.InvokeAsync(() =>
                {
                    UpdateAntennaDisplay();
                    UpdateMap();
                    UpdateRadarPosition();
                }, DispatcherPriority.Render);

                // PSO görselleştirmesini güncelle
                UpdatePsoVisualization();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Update timer error: {ex.Message}");
            }
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
            if (airplane == null) return;

            scanningAntenna.Latitude = antennaPosition.Lat;
            scanningAntenna.Longitude = antennaPosition.Lng;
            scanningAntenna.Altitude = 0;
            directionalAntenna.Latitude = antennaPosition.Lat;
            directionalAntenna.Longitude = antennaPosition.Lng;
            directionalAntenna.Altitude = 0;

            antennaController.UpdateScanningAntenna(scanningAntenna, airplane);
            antennaController.UpdateDirectionalAntenna(directionalAntenna, scanningAntenna, airplane);
        }

        private void AnimateProgressBar(ProgressBar bar, double newValue)
        {
            // Using exponential ease out for smoother animation
            DoubleAnimation animation = new DoubleAnimation
            {
                To = newValue,
                Duration = TimeSpan.FromMilliseconds(400), // Slightly longer duration
                EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
            };
            bar.BeginAnimation(ProgressBar.ValueProperty, animation);
        }

        private void UpdateAntennaDisplay()
        {
            Dispatcher.Invoke(() =>
            {
                try
                {
                    if (scanningAntenna != null)
                    {
                        ScanHAngle.Text = $"{scanningAntenna.HorizontalAngle:F1}°";
                        ScanVAngle.Text = $"{scanningAntenna.VerticalAngle:F1}°";
                        SignalStrength.Text = $"RSSI: {scanningAntenna.RSSI:F1} dBm\nSNR: {scanningAntenna.SNR:F1} dB";
                        ScanAreaSize.Text = $"Scan Area: {antennaController.CurrentScanArea:F1}°";
                        AnimateProgressBar(ScanSignalBar, scanningAntenna.SignalStrength);
                    }

                    if (directionalAntenna != null)
                    {
                        DirHAngle.Text = $"{directionalAntenna.HorizontalAngle:F1}°";
                        DirVAngle.Text = $"{directionalAntenna.VerticalAngle:F1}°";

                        // Directional antennanın RSSI/SNR değerlerini her zaman güncel göster (lock olsa da)
                        DirRssiValue.Text = $"{directionalAntenna.RSSI:F1}";
                        DirSnrValue.Text = $"{directionalAntenna.SNR:F1}";
                        AnimateProgressBar(DirSignalBar, directionalAntenna.SignalStrength);
                    }

                    // Update radar display
                    UpdateRadarDisplay();

                    // Status messages
                    string status = isScanning ?
                        (antennaController.IsInitialScan ? "Initial Scan" : "Fine Tracking") :
                        "Ready";

                    SystemStatus.Text = $"{status} | Scan Area: {antennaController.CurrentScanArea:F1}°";
                    StatusMessage.Text = $"Scanning: RSSI={scanningAntenna?.RSSI:F1}dBm | Tracking: RSSI={directionalAntenna?.RSSI:F1}dBm";

                    // Animate signal bars
                    AnimateProgressBar(ScanSignalBar, scanningAntenna?.SignalStrength ?? 0);
                    AnimateProgressBar(DirSignalBar, directionalAntenna?.SignalStrength ?? 0);
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"UpdateAntennaDisplay error: {ex.Message}");
                }
            });
        }

        private void UpdateRadarDisplay()
        {
            if (RadarSweep != null && scanningAntenna != null)
            {
                // Radar sweep rotation with animation
                if (sweepRotation != null)
                {
                    DoubleAnimation rotateAnim = new DoubleAnimation
                    {
                        To = scanningAntenna.HorizontalAngle,
                        Duration = TimeSpan.FromMilliseconds(400),
                        EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
                    };
                    sweepRotation.BeginAnimation(RotateTransform.AngleProperty, rotateAnim);
                }

                // Signal strength-based color and opacity animation
                byte targetAlpha = (byte)(scanningAntenna.SignalStrength * 1.5);
                Color targetColor = Color.FromArgb(targetAlpha, 0, 255, 0);

                SolidColorBrush currentBrush = RadarSweep.Fill as SolidColorBrush;
                if (currentBrush == null || currentBrush.Color != targetColor)
                {
                    ColorAnimation colorAnim = new ColorAnimation
                    {
                        To = targetColor,
                        Duration = TimeSpan.FromMilliseconds(400),
                        EasingFunction = FindResource("EaseOutExpo") as IEasingFunction
                    };
                    if (currentBrush == null)
                    {
                        currentBrush = new SolidColorBrush(targetColor);
                        RadarSweep.Fill = currentBrush;
                    }
                    currentBrush.BeginAnimation(SolidColorBrush.ColorProperty, colorAnim);
                }
            }

            // Signal bar güncelleme
            if (ScanSignalBar != null)
            {
                ScanSignalBar.Value = scanningAntenna?.SignalStrength ?? 0;
            }
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
                MapControl.Position = antennaPosition;

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
                MapControl.IsHitTestVisible = true;

                // Z-Index ayarları
                Panel.SetZIndex(MapControl, 0);
                Panel.SetZIndex(RadarElementsCanvas, 1);
                Panel.SetZIndex(ControlsCanvas, 2);

                // Canvas ayarları
                MapCanvas.Background = null;
                MapCanvas.IsHitTestVisible = true;

                RadarElementsCanvas.Background = null;
                RadarElementsCanvas.IsHitTestVisible = false;

                ControlsCanvas.Background = null;
                ControlsCanvas.IsHitTestVisible = true;

                // Ortalama butonu için özel ayar
                if (centerMapButton != null)
                {
                    centerMapButton.IsHitTestVisible = true;
                    Panel.SetZIndex(centerMapButton, 1000);
                }

                // Event handlers for map drag
                MapControl.MouseDown += (s, e) =>
                {
                    if (e.ChangedButton == MouseButton.Left)
                    {
                        MapControl.CaptureMouse();
                        e.Handled = true;
                    }
                };

                MapControl.MouseUp += (s, e) =>
                {
                    if (e.ChangedButton == MouseButton.Left)
                    {
                        MapControl.ReleaseMouseCapture();
                        e.Handled = true;
                    }
                };

                UpdateRadarPosition();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Harita başlatılamadı: {ex.Message}");
            }
        }

        private async Task UpdateRadarPosition()
        {
            if (MapControl == null) return;

            try
            {
                // Get antenna position - always center radar on antenna
                GPoint antennaPoint = MapControl.FromLatLngToLocal(antennaPosition);

                // Position radar elements around antenna
                if (RadarBackground != null)
                {
                    Canvas.SetLeft(RadarBackground, antennaPoint.X - RadarBackground.Width / 2);
                    Canvas.SetTop(RadarBackground, antennaPoint.Y - RadarBackground.Height / 2);
                }

                if (RadarGrid != null)
                {
                    Canvas.SetLeft(RadarGrid, antennaPoint.X - 100);
                    Canvas.SetTop(RadarGrid, antennaPoint.Y - 100);
                }

                if (RadarSweep != null)
                {
                    Canvas.SetLeft(RadarSweep, antennaPoint.X - 50);
                    Canvas.SetTop(RadarSweep, antennaPoint.Y - 50);
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
                // Tıklanan noktanın buton üzerinde olup olmadığını kontrol et
                if (e.Source is Button) return;

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
            // Panel genişliğini animasyonlu olarak değiştir
            DoubleAnimation animation = new DoubleAnimation
            {
                Duration = TimeSpan.FromMilliseconds(250),
                EasingFunction = new QuadraticEase()
            };

            if (rightPanelColumn.Width.Value > 0)
            {
                // Panel'i kapat
                animation.To = 0;
                TogglePanelButton.Content = "⟩";
            }
            else
            {
                // Panel'i aç
                animation.To = PANEL_WIDTH;
                TogglePanelButton.Content = "⟨";
            }

            // Grid sütun genişliğini animate et
            rightPanelColumn.BeginAnimation(ColumnDefinition.WidthProperty,
                new GridLengthAnimation
                {
                    From = rightPanelColumn.Width,
                    To = new GridLength(animation.To.Value),
                    Duration = animation.Duration
                });

            // Panel genişliğini animate et
            ControlPanel.BeginAnimation(FrameworkElement.WidthProperty, animation);
        }

        // Grid sütun genişliği için özel animasyon sınıfı
        public class GridLengthAnimation : AnimationTimeline
        {
            public static readonly DependencyProperty FromProperty =
                DependencyProperty.Register("From", typeof(GridLength), typeof(GridLengthAnimation));

            public static readonly DependencyProperty ToProperty =
                DependencyProperty.Register("To", typeof(GridLength), typeof(GridLengthAnimation));

            public GridLength From
            {
                get { return (GridLength)GetValue(FromProperty); }
                set { SetValue(FromProperty, value); }
            }

            public GridLength To
            {
                get { return (GridLength)GetValue(ToProperty); }
                set { SetValue(ToProperty, value); }
            }

            protected override Freezable CreateInstanceCore()
            {
                return new GridLengthAnimation();
            }

            public override Type TargetPropertyType
            {
                get { return typeof(GridLength); }
            }

            public override object GetCurrentValue(object defaultOriginValue, object defaultDestinationValue, AnimationClock animationClock)
            {
                double fromValue = ((GridLength)GetValue(FromProperty)).Value;
                double toValue = ((GridLength)GetValue(ToProperty)).Value;

                if (animationClock.CurrentProgress == null)
                    return null;

                double progress = animationClock.CurrentProgress.Value;
                return new GridLength((1 - progress) * fromValue + progress * toValue, GridUnitType.Pixel);
            }
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
            try
            {
                // Heading update (0-360 aralığında normalize et)
                heading = ((heading % 360) + 360) % 360;
                HeadingText.Text = $"HDG: {heading:000}°";

                // Altitude update (en yakın tam sayıya yuvarla)
                altitude = Math.Round(altitude, 0);
                AltitudeText.Text = $"ALT: {altitude:F0}m";

                // Ground Speed update (bir ondalık basamak)
                SpeedText.Text = $"GS: {speed:F1}m/s";

                // Battery update (bir ondalık basamak)
                BatteryText.Text = $"BAT: {battery:F1}V";
                BatteryIndicator.Value = Math.Max(0, Math.Min(100, battery * 10)); // 0-100 arasına normalize et

                // Throttle update (tam sayı)
                ThrottleText.Text = $"THR: {throttle:F0}%";
                ThrottleIndicator.Value = Math.Max(0, Math.Min(100, throttle));

                // Artificial horizon güncelleme
                if (airplane != null)
                {
                    UpdateArtificialHorizon(airplane.Pitch, airplane.Roll);
                }

                // Update pitch values
                if (airplane != null)
                {
                    double currentPitch = airplane.Pitch;

                    LeftPitchValue.Text = $"{currentPitch + 10:F0}° →";
                    RightPitchValue.Text = $"← {currentPitch + 10:F0}°";

                    LeftCurrentPitch.Text = $"{currentPitch:F0}° →";
                    RightCurrentPitch.Text = $"← {currentPitch:F0}°";

                    LeftLowerPitch.Text = $"{currentPitch - 10:F0}° →";
                    RightLowerPitch.Text = $"← {currentPitch - 10:F0}°";
                }

                // Altitude with arrow indicator
                string altitudeArrow = currentAltitude > 0 ? "↑" : currentAltitude < 0 ? "↓" : "→";
                AltitudeText.Text = $"ALT: {altitude:F0}m {altitudeArrow}";

                // Diğer HUD bileşenlerini güncelle
                UpdateCompassRose(heading);
                UpdateSpeedTape(speed);
                UpdateAltitudeTape(altitude);

                // System Status updates
                ScanMode.Text = $"Mode: {antennaController?.CurrentMode ?? "N/A"}";
                ScanProgress.Text = $"Progress: {(antennaController?.ScanProgress * 100):F0}%";
                PsoInfo.Text = $"PSO Iterations: {antennaController?.PsoIteration ?? 0}";
                ConvergenceInfo.Text = $"Conv: {(antennaController?.ConvergenceRate * 100):F0}%";
                SearchAreaInfo.Text = $"Area: {(antennaController?.CurrentScanArea ?? 360):F0}°";
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"HUD update error: {ex.Message}");
            }
        }

        private void UpdateCompassRose(double heading)
        {
            CompassRose.Children.Clear();
            CurrentHeadingText.Text = $"{heading:000}°";

            double centerAngle = ((int)heading + 360) % 360;
            int start = ((int)centerAngle - 45 + 360) % 360;
            int end = ((int)centerAngle + 45 + 360) % 360;

            for (int i = start; i <= start + 90; i += 30)
            {
                int displayAngle = ((i % 360) + 360) % 360;
                double adjustedAngle = (displayAngle - heading + 360) % 360;
                double x = 140 + 130 * Math.Sin(adjustedAngle * Math.PI / 180);

                if (x >= 0 && x <= 280)
                {
                    TextBlock tickLabel = new TextBlock
                    {
                        Text = displayAngle.ToString("000"),
                        Foreground = Brushes.White,
                        FontSize = 10
                    };

                    Canvas.SetLeft(tickLabel, x - 10);
                    Canvas.SetTop(tickLabel, displayAngle % 90 == 0 ? 5 : 10);
                    CompassRose.Children.Add(tickLabel);
                }
            }
        }

        private void UpdateArtificialHorizon(double pitch, double roll)
        {
            var transform = new TransformGroup();
            transform.Children.Add(new RotateTransform(roll, 75, 75));

            // Pitch değerlerini çizgilerle aynı hizaya getir
            double baselineY = 75; // Merkez çizginin Y koordinatı
            double pixelsPerDegree = 2.0; // Her derece için piksel değeri
            double topLineY = 45;    // Üst çizginin Y koordinatı
            double bottomLineY = 105; // Alt çizginin Y koordinatı

            // Üst pitch değerleri
            Canvas.SetLeft(LeftPitchValue, 5);
            Canvas.SetTop(LeftPitchValue, topLineY - 6); // -6 text height/2 için offset
            Canvas.SetRight(RightPitchValue, 5);
            Canvas.SetTop(RightPitchValue, topLineY - 6);

            // Orta pitch değerleri
            Canvas.SetLeft(LeftCurrentPitch, 5);
            Canvas.SetTop(LeftCurrentPitch, baselineY - 6);
            Canvas.SetRight(RightCurrentPitch, 5);
            Canvas.SetTop(RightCurrentPitch, baselineY - 6);

            // Alt pitch değerleri
            Canvas.SetLeft(LeftLowerPitch, 5);
            Canvas.SetTop(LeftLowerPitch, bottomLineY - 6);
            Canvas.SetRight(RightLowerPitch, 5);
            Canvas.SetTop(RightLowerPitch, bottomLineY - 6);

            // Pitch açısına göre kaydırma
            transform.Children.Add(new TranslateTransform(0, pitch * pixelsPerDegree));
            ArtificialHorizon.RenderTransform = transform;
        }

        private void UpdateSpeedTape(double speed)
        {
            SpeedTape.Children.Clear();
            int baseSpeed = (int)speed;

            // Mevcut hız göstergesi güncelleme
            CurrentSpeedText.Text = $"{speed:F0}";

            // Tape değerleri
            for (int i = -5; i <= 5; i++)
            {
                int tapeSpeed = baseSpeed + i * 5;
                if (tapeSpeed < 0 || i == 0) continue;

                var text = new TextBlock
                {
                    Text = tapeSpeed.ToString(),
                    Foreground = Brushes.White,
                    FontSize = 12,
                    Margin = new Thickness(5, 0, 0, 0)
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

            // Mevcut değer göstergesi güncelleme
            CurrentAltitudeText.Text = $"{altitude:F0}";
            AltitudeArrow.Text = altitude > 0 ? "↑" : altitude < 0 ? "↓" : "→";

            // Tape değerleri
            for (int i = -5; i <= 5; i++)
            {
                int tapeAlt = baseAlt + i * 10;
                if (i != 0) // Mevcut değeri atlayalım, zaten gösteriliyor
                {
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

        private void UpdatePsoVisualization()
        {
            try
            {
                PsoVisualizationCanvas.Children.Clear();

                // Grid çizgilerini çiz
                DrawPsoGrid();

                var psoState = antennaController?.GetPsoState();
                if (psoState == null) return;

                // Parçacık geçmişini çiz
                DrawParticleHistory(psoState.BestPositionHistory);

                // Search area indicator
                DrawSearchArea(psoState.SearchAreaH, psoState.SearchAreaV, psoState.BestPosition);

                // Parçacıkları çiz - fitness değerine göre renk ve boyut değişimi
                DrawParticles(psoState.Particles, psoState.ParticleFitness);

                // En iyi pozisyonu göster
                DrawBestPosition(psoState.BestPosition, psoState.BestFitness);

                // Convergence ve search radius göstergeleri
                UpdatePsoMetrics(psoState);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"PSO visualization error: {ex.Message}");
            }
        }

        private void DrawPsoGrid()
        {
            // Izgarayı daha belirgin hale getir
            for (int i = 0; i <= 180; i += 45)
            {
                var horizontalLine = new Line
                {
                    X1 = 0,
                    X2 = 180,
                    Y1 = i,
                    Y2 = i,
                    Stroke = new SolidColorBrush(Color.FromArgb(32, 255, 255, 255)),
                    StrokeThickness = i % 90 == 0 ? 2 : 1,
                    StrokeDashArray = new DoubleCollection { 4, 4 }
                };
                PsoVisualizationCanvas.Children.Add(horizontalLine);

                var verticalLine = new Line
                {
                    X1 = i,
                    X2 = i,
                    Y1 = 0,
                    Y2 = 180,
                    Stroke = new SolidColorBrush(Color.FromArgb(32, 255, 255, 255)),
                    StrokeThickness = i % 90 == 0 ? 2 : 1,
                    StrokeDashArray = new DoubleCollection { 4, 4 }
                };
                PsoVisualizationCanvas.Children.Add(verticalLine);
            }
        }

        private void DrawParticleHistory(Queue<(double H, double V)> history)
        {
            if (history.Count < 2) return;

            var points = new PointCollection();
            foreach (var pos in history)
            {
                points.Add(new Point(ScaleH(pos.H), ScaleV(pos.V)));
            }

            var historyPath = new System.Windows.Shapes.Polyline
            {
                Points = points,
                Stroke = new SolidColorBrush(Colors.Yellow),
                StrokeThickness = 1,
                Opacity = 0.3
            };

            PsoVisualizationCanvas.Children.Add(historyPath);
        }

        private void DrawSearchArea(double areaH, double areaV, (double H, double V) center)
        {
            var ellipse = new Ellipse
            {
                Width = ScaleH(areaH),
                Height = ScaleV(areaV),
                Stroke = new SolidColorBrush(Colors.LightBlue),
                StrokeThickness = 1,
                StrokeDashArray = new DoubleCollection { 2, 2 },
                Opacity = 0.3
            };

            Canvas.SetLeft(ellipse, ScaleH(center.H) - ellipse.Width / 2);
            Canvas.SetTop(ellipse, ScaleV(center.V) - ellipse.Height / 2);
            PsoVisualizationCanvas.Children.Add(ellipse);
        }

        private void DrawParticles(List<ParticleData> particles, List<double> fitness)
        {
            for (int i = 0; i < particles.Count; i++)
            {
                var particle = particles[i];
                double normalizedFitness = fitness[i] / fitness.Max();

                // Fitness değerine göre renk değişimi
                Color particleColor = ColorHelper.LerpColor(Colors.Red, Colors.LightGreen, normalizedFitness);

                var ellipse = new Ellipse
                {
                    Width = 6 + normalizedFitness * 4,
                    Height = 6 + normalizedFitness * 4,
                    Fill = new SolidColorBrush(particleColor),
                    Opacity = 0.8,
                    Effect = new System.Windows.Media.Effects.DropShadowEffect
                    {
                        Color = particleColor,
                        BlurRadius = 10,
                        ShadowDepth = 0
                    }
                };

                Canvas.SetLeft(ellipse, ScaleH(particle.HorizontalPosition) - ellipse.Width / 2);
                Canvas.SetTop(ellipse, ScaleV(particle.VerticalPosition) - ellipse.Height / 2);

                // Hız vektörü
                var velocityLine = new Line
                {
                    X1 = ScaleH(particle.HorizontalPosition),
                    Y1 = ScaleV(particle.VerticalPosition),
                    X2 = ScaleH(particle.HorizontalPosition + particle.HorizontalVelocity * 0.25),
                    Y2 = ScaleV(particle.VerticalPosition + particle.VerticalVelocity * 0.25),
                    Stroke = new SolidColorBrush(particleColor),
                    StrokeThickness = 1.5,
                    Opacity = 0.6
                };

                PsoVisualizationCanvas.Children.Add(velocityLine);
                PsoVisualizationCanvas.Children.Add(ellipse);
            }
        }

        private void DrawBestPosition((double H, double V) bestPosition, double fitness)
        {
            var bestMarker = new Ellipse
            {
                Width = 12,
                Height = 12,
                Fill = new SolidColorBrush(Colors.Lime),
                Stroke = new SolidColorBrush(Colors.White),
                StrokeThickness = 2,
                Effect = new System.Windows.Media.Effects.DropShadowEffect
                {
                    Color = Colors.Lime,
                    BlurRadius = 15,
                    ShadowDepth = 0
                }
            };

            Canvas.SetLeft(bestMarker, ScaleH(bestPosition.H) - 6);
            Canvas.SetTop(bestMarker, ScaleV(bestPosition.V) - 6);
            PsoVisualizationCanvas.Children.Add(bestMarker);
        }

        private void UpdatePsoMetrics(PsoState state)
        {
            // Update metrics display
            BestPositionH.Text = $"{state.BestPosition.H:F1}°";
            BestPositionV.Text = $"{state.BestPosition.V:F1}°";

            double convergenceValue = (state.ConvergenceRate) * 100;
            ConvergenceBar.Value = convergenceValue;
            ConvergenceText.Text = $"{convergenceValue:F0}%";

            SearchAreaText.Text = $"H: {state.SearchAreaH:F1}° × V: {state.SearchAreaV:F1}°";
            ParticleCountText.Text = state.ParticleCount.ToString();
        }

        private double ScaleH(double angle)
        {
            angle = Math.Max(0, Math.Min(360, angle));
            return (angle / 360.0) * 180;
        }

        private double ScaleV(double angle)
        {
            angle = Math.Max(0, Math.Min(90, angle));
            return (angle / 90.0) * 180;
        }
    }
}
