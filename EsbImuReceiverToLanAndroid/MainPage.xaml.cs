using Microsoft.Maui.Controls;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using EsbReceiverToLanAndroid.Platforms.Android.Services;
using SlimeImuProtocol.SlimeVR;

namespace EsbReceiverToLanAndroid;

public partial class MainPage : ContentPage {
    private bool _isTrackerServiceStarted = false;

    public MainPage() {
        InitializeComponent();
        BuildUI();
        LoadConfig();
        titleLabel.Text = "ESB IMU Receiver To Lan";
        _ = typeof(TrackerUsbReceiver);
    }

    private void StartButton_Clicked(object sender, EventArgs e) {
        if (!_isTrackerServiceStarted) {
            string endpoint = ipEntry.Text;
            if (IPAddress.TryParse(endpoint, out _)) {
                UDPHandler.Endpoint = endpoint;
                File.WriteAllText(Path.Combine(FileSystem.AppDataDirectory, "config.txt"), endpoint);
                statusLabel.Text = "Receiver started.";
                statusLabel.TextColor = Colors.Green;
                // Start UDP listener
                var context = Android.App.Application.Context;
                var intent = new Android.Content.Intent(context, typeof(Platforms.Android.Services.TrackerListenerService));

                if (Android.OS.Build.VERSION.SdkInt >= Android.OS.BuildVersionCodes.O)
                    context.StartForegroundService(intent);
                else
                    context.StartService(intent);
                _isTrackerServiceStarted = true;
            } else {
                statusLabel.Text = "Invalid IP address.";
                statusLabel.TextColor = Colors.Red;
            }
        }
    }

    
    // Method to get the local IP address of the device
    private string GetLocalIPAddress() {
        string ipAddress = "Unable to retrieve IP";
        try {
            // Platform-specific logic to retrieve the device's IP address
#if ANDROID
            ipAddress = GetAndroidLocalIPAddress();
#elif IOS
            ipAddress = GetIosLocalIPAddress();
#else
            ipAddress = GetLocalIPAddressFallback();
#endif
        } catch (Exception ex) {
            Debug.WriteLine($"Error retrieving IP: {ex.Message}");
            ipAddress = "Error retrieving IP address";
        }
        return ipAddress;
    }

    // Android-specific method for getting IP address
    private string GetAndroidLocalIPAddress() {
        string ipAddress = "Unable to retrieve IP";
        try {
            var wifiManager = (Android.Net.Wifi.WifiManager)Android.App.Application.Context.GetSystemService(Android.Content.Context.WifiService);
            var wifiInfo = wifiManager.ConnectionInfo;

            if (wifiInfo == null) {
                return "No WiFi Connection";
            }

            var ip = wifiInfo.IpAddress;
            ipAddress = string.Format("{0}.{1}.{2}.{3}",
                (ip & 0xFF),
                (ip >> 8 & 0xFF),
                (ip >> 16 & 0xFF),
                (ip >> 24 & 0xFF));

            Debug.WriteLine($"Android IP Address: {ipAddress}");
        } catch (Exception ex) {
            Debug.WriteLine($"Android IP Error: {ex.Message}");
        }
        return ipAddress;
    }
    private void OnVibrateClicked(object sender, EventArgs e) {
        TrackerListenerService.Instance.Vibrate();
    }

    // iOS-specific method for getting IP address
    private string GetIosLocalIPAddress() {
        string ipAddress = "Unable to retrieve IP";
        try {
            var networkInterfaces = System.Net.NetworkInformation.NetworkInterface.GetAllNetworkInterfaces();
            foreach (var ni in networkInterfaces) {
                if (ni.OperationalStatus == System.Net.NetworkInformation.OperationalStatus.Up) {
                    foreach (var ip in ni.GetIPProperties().UnicastAddresses) {
                        if (ip.Address.AddressFamily == AddressFamily.InterNetwork) {
                            ipAddress = ip.Address.ToString();
                            Debug.WriteLine($"iOS IP Address: {ipAddress}");
                            return ipAddress;
                        }
                    }
                }
            }
        } catch (Exception ex) {
            Debug.WriteLine($"iOS IP Error: {ex.Message}");
        }
        return ipAddress;
    }

    // Fallback for other platforms (Windows/macOS) to get IP
    private string GetLocalIPAddressFallback() {
        string ipAddress = "Unable to retrieve IP";
        try {
            var host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (var ip in host.AddressList) {
                if (ip.AddressFamily == AddressFamily.InterNetwork) {
                    ipAddress = ip.ToString();
                    Debug.WriteLine($"Fallback IP Address: {ipAddress}");
                    break;
                }
            }
        } catch (Exception ex) {
            Debug.WriteLine($"Fallback IP Error: {ex.Message}");
        }
        return ipAddress;
    }
    private void BuildUI() {
        ipEntry = new Entry { Placeholder = "Enter destination IP" };
        statusLabel = new Label { Text = "", TextColor = Colors.Red };
        startButton = new Button { Text = "Start Receiver" };
        startButton.Clicked += StartButton_Clicked;

        Content = new VerticalStackLayout {
            Padding = 20,
            Children =
            {
                new Label { Text = "ESB IMU Receiver to LAN", FontSize = 24, HorizontalOptions = LayoutOptions.Center },
                ipEntry,
                startButton,
                statusLabel
            }
        };
    }

    private void LoadConfig() {
        string configPath = Path.Combine(FileSystem.AppDataDirectory, "config.txt");
        if (File.Exists(configPath)) {
            ipEntry.Text = File.ReadAllText(configPath);
        }
    }
}
