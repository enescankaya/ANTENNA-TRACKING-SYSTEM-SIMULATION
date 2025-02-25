using System;
using System.Net.Sockets;
using System.Threading.Tasks;
using Project.Models;

namespace Project.Services
{
    public class SITLConnection
    {
        private TcpClient client;
        private NetworkStream stream;
        private bool isConnected;

        public event EventHandler<AirplaneState> OnPositionUpdate;
        public event EventHandler<string> OnConnectionStatusChanged;

        public async Task Connect(string host = "127.0.0.1", int port = 5762)
        {
            try
            {
                client = new TcpClient();
                await client.ConnectAsync(host, port);
                stream = client.GetStream();
                isConnected = true;
                OnConnectionStatusChanged?.Invoke(this, "Connected");
                StartListening();
            }
            catch (Exception ex)
            {
                OnConnectionStatusChanged?.Invoke(this, $"No Connection");
                throw;
            }
        }

        private async void StartListening()
        {
            while (isConnected)
            {
                try
                {
                    byte[] buffer = new byte[1024];
                    int bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length);
                    if (bytesRead > 0)
                    {
                        await ProcessMavlinkMessage(buffer, bytesRead);
                    }
                }
                catch (Exception ex)
                {
                    OnConnectionStatusChanged?.Invoke(this, $"No Connection");
                    break;
                }
            }
        }

        private async Task ProcessMavlinkMessage(byte[] buffer, int bytesRead)
        {
            try
            {
                // Simple parsing of position data (in real implementation, use proper MAVLink parsing)
                var state = new AirplaneState
                {
                    // Simulated values for testing
                    Latitude = Math.Sin(DateTime.Now.Second * Math.PI / 30) * 0.001,
                    Longitude = Math.Cos(DateTime.Now.Second * Math.PI / 30) * 0.001,
                    Altitude = 100 + Math.Sin(DateTime.Now.Second * Math.PI / 15) * 10,
                    Heading = (DateTime.Now.Second * 6) % 360
                };
                OnPositionUpdate?.Invoke(this, state);
            }
            catch (Exception ex)
            {
                OnConnectionStatusChanged?.Invoke(this, $"Message processing error: {ex.Message}");
            }
        }

        public void Disconnect()
        {
            isConnected = false;
            stream?.Close();
            client?.Close();
            OnConnectionStatusChanged?.Invoke(this, "Disconnected");
        }
    }
}
