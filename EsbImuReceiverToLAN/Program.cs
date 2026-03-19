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

            if (string.IsNullOrEmpty(savedEndpoint) || savedEndpoint == "255.255.255.255") {
                Console.WriteLine("Searching for SlimeVR server on local network...");
                
                UDPHandler.OnServerDiscovered += (sender, ip) => {
                    if (_discoveryDone.WaitOne(0)) return; // Already found
                    
                    Console.WriteLine($"\nSlimeVR server discovered at: {ip}");
                    UDPHandler.Endpoint = ip;
                    File.WriteAllText(configPath, ip);
                    _discoveryDone.Set();
                };

                // Perform one manual check/broadcast
                UDPHandler.Endpoint = "255.255.255.255";
                
                // Wait for discovery or user input
                Console.WriteLine("Waiting for discovery (or press Enter to enter IP manually)...");
                
                // Use a separate thread to handle manual entry if discovery is taking too long
                Task.Run(() => {
                    string? manualIp = Console.ReadLine();
                    if (!string.IsNullOrEmpty(manualIp) && IPAddress.TryParse(manualIp, out _)) {
                        UDPHandler.Endpoint = manualIp;
                        File.WriteAllText(configPath, manualIp);
                        _discoveryDone.Set();
                    }
                });

                _discoveryDone.WaitOne();
            } else {
                UDPHandler.Endpoint = savedEndpoint;
                Console.WriteLine($"Using saved destination: {savedEndpoint}");
            }

            Console.WriteLine("Initializing HID trackers...");
            _trackersHid = new TrackersHID();
            
            while (true) {
                Thread.Sleep(10000);
            }
        }
    }
}
