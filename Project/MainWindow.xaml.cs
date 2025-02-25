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
using System.Threading.Tasks;

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
        private PointLatLng baseStationPosition;
        private bool isDragging = false;
        private Point dragStart;

        // Görsel elemanlar için
        private Ellipse planeMarker;
        private Line directionLine;
        private Path radarSweep;
        private TranslateTransform sweepPosition;
        private RotateTransform sweepRotation;
        private DateTime lastMapCenterUpdate = DateTime.MinValue;
        private const int MAP_CENTER_INTERVAL = 5000; // 5 seconds in milliseconds

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
        }

        private void InitializeComponents()
        {
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
        }

        private void InitializeTimer()
        {
            updateTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100)
            };
            updateTimer.Tick += UpdateTimer_Tick;
        }

        private void InitializeVisuals()
        {
            // Radar sweep oluştur
            radarSweep = new Path
            {
                Fill = new SolidColorBrush(Color.FromArgb(64, 0, 255, 0)),
                Stroke = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0)),
                StrokeThickness = 2
            };

            // Radar sweep geometrisi
            var geometry = new PathGeometry();
            var figure = new PathFigure();
            figure.StartPoint = new Point(0, 0);
            figure.Segments.Add(new LineSegment(new Point(150, 0), true));
            figure.Segments.Add(new ArcSegment(
                new Point(0, 0),
                new Size(150, 150),
                45, // Sweep angle
                false, // IsLargeArc
                SweepDirection.Clockwise,
                true)); // IsStroked
            geometry.Figures.Add(figure);
            radarSweep.Data = geometry;

            // Transform'ları oluştur
            sweepRotation = new RotateTransform();
            sweepPosition = new TranslateTransform();

            var transformGroup = new TransformGroup();
            transformGroup.Children.Add(sweepRotation);
            transformGroup.Children.Add(sweepPosition);
            radarSweep.RenderTransform = transformGroup;

            MapCanvas.Children.Add(radarSweep);

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

            // Başlangıç pozisyonları
            Canvas.SetLeft(planeMarker, MapCanvas.ActualWidth / 2);
            Canvas.SetTop(planeMarker, MapCanvas.ActualHeight / 2);
        }

        private void UpdateMap()
        {
            if (airplane == null || planeMarker == null || directionLine == null || MapCanvas == null)
                return;

            try
            {
                // Only center map every 5 seconds
                bool shouldCenterMap = (DateTime.Now - lastMapCenterUpdate).TotalMilliseconds >= MAP_CENTER_INTERVAL;

                if (shouldCenterMap)
                {
                    MapControl.Position = new PointLatLng(airplane.Latitude, airplane.Longitude);
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
                directionLine.Y2 = screenPoint.Y + length * Math.Sin(angleRad);

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
                await sitlConnection.Connect();
                ConnectionStatus.Text = "Connected";
                ConnectButton.IsEnabled = false;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Connection failed: {ex.Message}");
            }
        }

        private void StartButton_Click(object sender, RoutedEventArgs e)
        {
            isScanning = !isScanning;
            StartButton.Content = isScanning ? "Stop Scanning" : "Start Scanning";

            if (isScanning)
                updateTimer.Start();
            else
                updateTimer.Stop();
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
            airplane = e;
            if (isScanning)
            {
                UpdateAntennaPositions();
            }
            UpdateAntennaDisplay();
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
            // Update angles
            ScanHAngle.Text = $"{scanningAntenna.HorizontalAngle:F1}°";
            ScanVAngle.Text = $"{scanningAntenna.VerticalAngle:F1}°";
            DirHAngle.Text = $"{directionalAntenna.HorizontalAngle:F1}°";
            DirVAngle.Text = $"{directionalAntenna.VerticalAngle:F1}°";

            // Update signal strength
            double normalizedStrength = scanningAntenna.SignalStrength / 100.0;
            SignalScale.ScaleX = normalizedStrength;
            SignalStrength.Text = $"{scanningAntenna.SignalStrength:F1}%";

            // Update radar sweep
            if (radarRotation != null)
            {
                radarRotation.Angle = scanningAntenna.HorizontalAngle;
            }

            // Update map
            UpdateMap();
        }

        private void InitializeMap()
        {
            try
            {
                // Cache ve proxy ayarları
                GMaps.Instance.Mode = AccessMode.ServerAndCache;
                GMap.NET.MapProviders.GMapProvider.WebProxy = System.Net.WebRequest.GetSystemWebProxy();
                GMap.NET.MapProviders.GMapProvider.WebProxy.Credentials = System.Net.CredentialCache.DefaultCredentials;

                // OpenStreetMap provider'ı kullan
                MapControl.MapProvider = GMap.NET.MapProviders.OpenStreetMapProvider.Instance;

                // Keşan Havalimanı koordinatları
                baseStationPosition = new PointLatLng(40.737222, 26.571667);
                MapControl.Position = baseStationPosition;

                // Temel ayarlar
                MapControl.MinZoom = 2;
                MapControl.MaxZoom = 17;
                MapControl.Zoom = 14; // Zoom seviyesini biraz artırdık

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
            if (MapControl == null) return;

            await Dispatcher.InvokeAsync(() =>
            {
                try
                {
                    // Baz istasyonunun ekran koordinatlarını al
                    GPoint baseStationPoint = MapControl.FromLatLngToLocal(baseStationPosition);

                    // Radar pozisyonunu güncelle
                    if (sweepPosition != null)
                    {
                        sweepPosition.X = baseStationPoint.X;
                        sweepPosition.Y = baseStationPoint.Y;
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"Radar position update error: {ex.Message}");
                }
            });
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
            if (radarSweep != null && scanningAntenna != null)
            {
                if (sweepRotation != null)
                {
                    sweepRotation.Angle = scanningAntenna.HorizontalAngle;
                }

                if (MapControl != null && baseStationPosition != null && sweepPosition != null)
                {
                    var point = MapControl.FromLatLngToLocal(baseStationPosition);
                    sweepPosition.X = point.X;
                    sweepPosition.Y = point.Y;
                }

                // Sinyal gücüne göre renk ayarla
                byte alpha = (byte)(scanningAntenna.SignalStrength * 2.55);
                radarSweep.Fill = new SolidColorBrush(Color.FromArgb(alpha, 0, 255, 0));
            }
        }
    }
}
