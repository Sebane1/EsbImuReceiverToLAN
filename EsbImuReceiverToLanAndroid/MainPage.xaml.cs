using Android.Content;
using EsbReceiverToLanAndroid.Platforms.Android.Services;
using Google.Android.Material.Slider;
using SlimeImuProtocol;
using SlimeImuProtocol.SlimeVR;
using System.Net;
using Slider = Microsoft.Maui.Controls.Slider;

namespace EsbReceiverToLanAndroid;

public partial class MainPage : ContentPage
{
    private bool _isTrackerServiceStarted = false;
    private Intent intent;
    public MainPage()
    {
        InitializeComponent();
        BuildUI();
        LoadConfig();
        titleLabel.Text = "ESB IMU Receiver To Lan";
        _ = typeof(TrackerUsbReceiver);
        TrackerUsbReceiver.OnDeviceConnected += delegate
        {
            startButton.Text = "Stop Receiver";
            statusLabel.Text = "Receiver started.";
            statusLabel.TextColor = Colors.Green;
            _isTrackerServiceStarted = true;
        };
        TrackerUsbReceiver.OnDeviceDisconnected += delegate
        {
            startButton.Text = "Start Receiver";
            statusLabel.Text = "Receiver stopped.";
            statusLabel.TextColor = Colors.Orange;
            _isTrackerServiceStarted = false;
        };
    }

    private void StartButton_Clicked(object? sender, EventArgs e)
    {
        var context = Android.App.Application.Context;


        if (!_isTrackerServiceStarted)
        {
            intent = new Android.Content.Intent(context, typeof(Platforms.Android.Services.TrackerListenerService));
            string endpoint = ipEntry.Text;
            if (IPAddress.TryParse(endpoint, out _))
            {
                UDPHandler.Endpoint = endpoint;
                File.WriteAllText(Path.Combine(FileSystem.AppDataDirectory, "config.txt"), endpoint);
                statusLabel.Text = "Receiver started.";
                statusLabel.TextColor = Colors.Green;

                // Start service
                if (Android.OS.Build.VERSION.SdkInt >= Android.OS.BuildVersionCodes.O)
                    context.StartForegroundService(intent);
                else
                    context.StartService(intent);

                _isTrackerServiceStarted = true;
                startButton.Text = "Stop Receiver";
            }
            else
            {
                statusLabel.Text = "Invalid IP address.";
                statusLabel.TextColor = Colors.Red;
            }
        }
        else
        {
            TrackerListenerService.Instance?.StopTrackerWork();
            TrackerListenerService.Instance?.StopSelf();
            // Stop the service
            try
            {
                context?.StopService(intent);
            }
            catch
            {
            }
            _isTrackerServiceStarted = false;
            startButton.Text = "Start Receiver";
            statusLabel.Text = "Receiver stopped.";
            statusLabel.TextColor = Colors.Orange;
        }
    }


    private void RefreshButton_Clicked(object? sender, EventArgs e)
    {
        UDPHandler.ForceUDPClientsToDoHandshake();
    }

    private void OnVibrateClicked(object sender, EventArgs e)
    {
        TrackerListenerService.Instance.Vibrate();
    }

    private void BuildUI()
    {
        ipEntry = new Entry { Placeholder = "Enter destination IP" };
        statusLabel = new Label { Text = "", TextColor = Colors.Red };
        startButton = new Button { Text = "Start Receiver" };
        startButton.Clicked += StartButton_Clicked;
        refreshButton = new Button { Text = "Refresh Trackers" };
        refreshButton.Clicked += RefreshButton_Clicked;

        Content = new VerticalStackLayout
        {
            Padding = 20,
            Children =
            {
                new Label { Text = "ESB IMU Receiver to LAN", FontSize = 24, HorizontalOptions = LayoutOptions.Center },
                ipEntry,
                startButton,
                refreshButton,
                statusLabel
            }
        };
    }

    private void LoadConfig()
    {
        string configPath = Path.Combine(FileSystem.AppDataDirectory, "config.txt");
        if (File.Exists(configPath))
            ipEntry.Text = File.ReadAllText(configPath);

        string ratePath = Path.Combine(FileSystem.AppDataDirectory, "rate.txt");
        if (File.Exists(ratePath) && int.TryParse(File.ReadAllText(ratePath), out int packetsAllowedPerSecond))
        {
            rateSlider.Value = packetsAllowedPerSecond;
        }
    }
}
