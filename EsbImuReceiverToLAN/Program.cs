using Everything_To_IMU_SlimeVR.Utility;
using EsbImuReceiverToLan.Tracking.Trackers.HID;
using Everything_To_IMU_SlimeVR.SlimeVR;
using System.Net;

namespace EspImuReceiverToLAN {
    internal class Program {
        static TrackersHID _trackersHid;
        static void Main(string[] args) {
            string configPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "config.txt");
            if (!File.Exists(configPath)) {
                while (true) {
                    Console.WriteLine("Please enter destination ip.");
                    string endpoint = Console.ReadLine();
                    if (IPAddress.TryParse(endpoint, out _)) {
                        UDPHandler.Endpoint = endpoint;
                        File.WriteAllText(configPath, endpoint);
                        break;
                    } else {
                        Console.WriteLine("Invalid destination!");
                    }
                }
            } else {
                UDPHandler.Endpoint = File.ReadAllText(configPath);
            }
            _trackersHid = new TrackersHID();
            while (true) {
                Thread.Sleep(10000);
            }
        }
    }
}
