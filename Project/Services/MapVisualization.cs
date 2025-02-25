using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using Project.Models;

namespace Project.Services
{
    public class MapVisualization
    {
        private readonly Canvas mapCanvas;
        private Ellipse airplaneMarker;
        private Line scanningAntennaLine;
        private Line directionalAntennaLine;
        private const double SCALE_FACTOR = 1.0;

        public MapVisualization(Canvas canvas)
        {
            mapCanvas = canvas;
            InitializeComponents();
        }

        private void InitializeComponents()
        {
            // Create airplane marker
            airplaneMarker = new Ellipse
            {
                Width = 10,
                Height = 10,
                Fill = Brushes.Red
            };
            mapCanvas.Children.Add(airplaneMarker);

            // Create scanning antenna line
            scanningAntennaLine = new Line
            {
                Stroke = Brushes.Blue,
                StrokeThickness = 2
            };
            mapCanvas.Children.Add(scanningAntennaLine);

            // Create directional antenna line
            directionalAntennaLine = new Line
            {
                Stroke = Brushes.Green,
                StrokeThickness = 2
            };
            mapCanvas.Children.Add(directionalAntennaLine);
        }

        public void UpdateAirplanePosition(AirplaneState airplane)
        {
            Point position = ConvertGPSToCanvas(airplane.Latitude, airplane.Longitude);
            Canvas.SetLeft(airplaneMarker, position.X - airplaneMarker.Width / 2);
            Canvas.SetTop(airplaneMarker, position.Y - airplaneMarker.Height / 2);
        }

        public void UpdateAntennas(AntennaState scanning, AntennaState directional)
        {
            // Update scanning antenna visualization
            Point center = new Point(mapCanvas.ActualWidth / 2, mapCanvas.ActualHeight / 2);
            double length = 100;

            // Scanning antenna
            double scanRadians = scanning.HorizontalAngle * Math.PI / 180;
            scanningAntennaLine.X1 = center.X;
            scanningAntennaLine.Y1 = center.Y;
            scanningAntennaLine.X2 = center.X + length * Math.Cos(scanRadians);
            scanningAntennaLine.Y2 = center.Y + length * Math.Sin(scanRadians);

            // Directional antenna
            double dirRadians = directional.HorizontalAngle * Math.PI / 180;
            directionalAntennaLine.X1 = center.X;
            directionalAntennaLine.Y1 = center.Y;
            directionalAntennaLine.X2 = center.X + length * Math.Cos(dirRadians);
            directionalAntennaLine.Y2 = center.Y + length * Math.Sin(dirRadians);
        }

        private Point ConvertGPSToCanvas(double lat, double lon)
        {
            // Simple conversion for demo purposes
            // In real application, use proper GPS to canvas coordinate conversion
            double x = (lon + 180) * mapCanvas.ActualWidth / 360;
            double y = (90 - lat) * mapCanvas.ActualHeight / 180;
            return new Point(x, y);
        }

        public void DrawSignalStrength(double strength, Point position)
        {
            Ellipse signalIndicator = new Ellipse
            {
                Width = 5,
                Height = 5,
                Fill = new SolidColorBrush(Color.FromArgb(
                    (byte)(strength * 2.55),
                    0,
                    255,
                    0))
            };

            Canvas.SetLeft(signalIndicator, position.X);
            Canvas.SetTop(signalIndicator, position.Y);
            mapCanvas.Children.Add(signalIndicator);
        }
    }
}
