using SlimeImuProtocol.Utility;
using EsbImuReceiverToLan.Tracking.Trackers.HID;
using SlimeImuProtocol.SlimeVR;
using System.Net;

namespace EspImuReceiverToLAN {
    internal class Program {
        static TrackersHID _trackersHid;
        static ManualResetEvent _discoveryDone = new ManualResetEvent(false);

        static void Main(string[] args) {
            string configPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "config.txt");
            string? savedEndpoint = File.Exists(configPath) ? File.ReadAllText(configPath).Trim() : null;

            if (!string.IsNullOrEmpty(savedEndpoint) && savedEndpoint != "255.255.255.255") {
                UDPHandler.Endpoint = savedEndpoint;
                Console.WriteLine($"Using saved destination: {savedEndpoint}");
            } else {
                UDPHandler.Endpoint = "255.255.255.255";
                Console.WriteLine("Searching for SlimeVR server on local network...");
                
                UDPHandler.OnServerDiscovered += (sender, ip) => {
                    if (_discoveryDone.WaitOne(0)) return; // Already found
                    
                    Console.WriteLine($"\n[Discovery] SlimeVR server discovered at: {ip}");
                    UDPHandler.Endpoint = ip;
                    File.WriteAllText(configPath, ip);
                    _discoveryDone.Set();
                };
            }

            Console.WriteLine("Initializing HID trackers...");
            _trackersHid = new TrackersHID();

            if (UDPHandler.Endpoint == "255.255.255.255") {
                Console.WriteLine("Waiting for discovery (or type server IP and press Enter)...");
                
                Task.Run(() => {
                    string? manualIp = Console.ReadLine();
                    if (!string.IsNullOrEmpty(manualIp) && IPAddress.TryParse(manualIp, out _)) {
                        UDPHandler.Endpoint = manualIp;
                        File.WriteAllText(configPath, manualIp);
                        _discoveryDone.Set();
                        Console.WriteLine($"Set manual destination: {manualIp}");
                    }
                });
            }

            while (true) {
                Thread.Sleep(10000);
            }
        }
    }
}
