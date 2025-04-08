using System;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.IO;
using Project.Models;

namespace Project.Services
{
    public class SITLConnection
    {
        private TcpClient client;
        private NetworkStream stream;
        private bool isConnected;
        private readonly MAVLink.MavlinkParse mavlinkParser = new MAVLink.MavlinkParse();
        private AirplaneState currentState;

        public event EventHandler<AirplaneState> OnPositionUpdate;
        public event EventHandler<string> OnConnectionStatusChanged;

        public async Task Connect(string host = "127.0.0.1", int port = 5760)
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
                OnConnectionStatusChanged?.Invoke(this, $"Connection failed: {ex.Message}");
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
                using (var memStream = new MemoryStream(buffer, 0, bytesRead))
                {
                    var message = mavlinkParser.ReadPacket(memStream);
                    if (message == null) return;

                    // Eğer currentState null ise yeni oluştur
                    if (currentState == null) currentState = new AirplaneState();

                    switch (message.msgid)
                    {
                        case (uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT:
                            var pos = (MAVLink.mavlink_global_position_int_t)message.data;
                            currentState.Latitude = pos.lat / 1E7;
                            currentState.Longitude = pos.lon / 1E7;
                            currentState.Altitude = pos.relative_alt / 1000.0f;
                            currentState.Heading = pos.hdg / 100.0f;
                            break;

                        case (uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE:
                            var att = (MAVLink.mavlink_attitude_t)message.data;
                            currentState.Pitch = att.pitch * 180.0f / (float)Math.PI;
                            currentState.Roll = att.roll * 180.0f / (float)Math.PI;
                            break;

                        case (uint)MAVLink.MAVLINK_MSG_ID.VFR_HUD:
                            var hud = (MAVLink.mavlink_vfr_hud_t)message.data;
                            currentState.AirSpeed = hud.airspeed;
                            currentState.GroundSpeed = hud.groundspeed;
                            currentState.ClimbRate = hud.climb;
                            currentState.Throttle = hud.throttle;
                            break;

                        case (uint)MAVLink.MAVLINK_MSG_ID.BATTERY_STATUS:
                            var bat = (MAVLink.mavlink_battery_status_t)message.data;
                            currentState.Battery = bat.voltages[0] / 1000.0f;
                            break;
                    }

                    // Her mesaj sonrası state'i gönder
                    OnPositionUpdate?.Invoke(this, currentState);
                }
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

        public bool IsConnected => isConnected;
    }
}
