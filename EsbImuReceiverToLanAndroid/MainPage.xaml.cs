using Android.Content;
using EsbReceiverToLanAndroid.Models;
using Microsoft.Maui.ApplicationModel;
using EsbReceiverToLanAndroid.Platforms.Android.Services;
using EsbReceiverToLanAndroid.Views;
using SlimeImuProtocol.SlimeVR;
using System.Net;

namespace EsbReceiverToLanAndroid;

public partial class MainPage : ContentPage
{
    private bool _isTrackerServiceStarted;
    private Intent? intent;
    private IDispatcherTimer? _refreshTimer;
    private string _lastTopologySignature = "";
    private readonly List<(TrackerRotationView RotView, Label NameLabel, Label InfoLabel)> _trackerRowCache = new();

    public MainPage()
    {
        InitializeComponent();
        LoadConfig();
        _ = typeof(TrackerUsbReceiver);
        TrackerUsbReceiver.OnDeviceConnected += OnDeviceConnected;
        TrackerUsbReceiver.OnDeviceDisconnected += OnDeviceDisconnected;
    }

    protected override void OnAppearing()
    {
        base.OnAppearing();
        _refreshTimer = Dispatcher.CreateTimer();
        _refreshTimer.Interval = TimeSpan.FromMilliseconds(50); // 20 Hz for smoother rotation preview
        _refreshTimer.Tick += (_, _) => RefreshTrackerList();
        _refreshTimer.Start();
    }

    protected override void OnDisappearing()
    {
        _refreshTimer?.Stop();
        _refreshTimer = null;
        base.OnDisappearing();
    }

    private void OnDeviceConnected(object? sender, EventArgs e)
    {
        MainThread.BeginInvokeOnMainThread(() =>
        {
            startButton.Text = "Stop";
            statusLabel.Text = "Receiving...";
            statusLabel.TextColor = Colors.LightGreen;
            _isTrackerServiceStarted = true;
        });
    }

    private void OnDeviceDisconnected(object? sender, EventArgs e)
    {
        MainThread.BeginInvokeOnMainThread(() =>
        {
            startButton.Text = "Start";
            statusLabel.Text = "Stopped";
            statusLabel.TextColor = Colors.Orange;
            _isTrackerServiceStarted = false;
        });
    }

    private void RefreshTrackerList()
    {
        var snapshot = TrackerListenerService.Instance?.GetTrackerSnapshot();
        MainThread.BeginInvokeOnMainThread(() => UpdateTrackerUI(snapshot));
    }

    private void UpdateTrackerUI(TrackerSnapshot? snapshot)
    {
        if (snapshot == null || snapshot.Dongles.Count == 0)
        {
            if (_lastTopologySignature != "")
            {
                _lastTopologySignature = "";
                _trackerRowCache.Clear();
                trackerListContainer.Children.Clear();
            }
            trackerSummaryLabel.Text = "No trackers connected";
            return;
        }

        var total = snapshot.Dongles.Sum(d => d.Trackers.Count);
        trackerSummaryLabel.Text = $"{total} tracker(s) across {snapshot.Dongles.Count} dongle(s)";

        var orderedTrackers = snapshot.Dongles
            .SelectMany(d => d.Trackers.OrderBy(t => t.Id).Select(t => (Dongle: d, Tracker: t)))
            .ToList();
        var signature = string.Join("|", snapshot.Dongles.Select(d => d.DeviceKey + ":" + string.Join(",", d.Trackers.OrderBy(t => t.Id).Select(t => t.Id))));

        if (signature != _lastTopologySignature)
        {
            _lastTopologySignature = signature;
            RebuildTrackerList(snapshot, orderedTrackers);
        }
        else
        {
            for (int i = 0; i < orderedTrackers.Count && i < _trackerRowCache.Count; i++)
            {
                var (_, t) = orderedTrackers[i];
                var (rotView, nameLabel, infoLabel) = _trackerRowCache[i];
                rotView.TrackerRotation = t.Rotation;
                nameLabel.Text = t.DisplayName;
                infoLabel.Text = $"{t.BatteryLevel:F0}% • {t.Status}";
            }
        }
    }

    private void RebuildTrackerList(TrackerSnapshot snapshot, List<(DongleGroup Dongle, TrackerInfo Tracker)> orderedTrackers)
    {
        _trackerRowCache.Clear();
        trackerListContainer.Children.Clear();

        var dongleGroups = orderedTrackers.GroupBy(x => x.Dongle.DeviceKey).ToList();
        foreach (var grp in dongleGroups)
        {
            var dongle = grp.First().Dongle;
            var dongleHeader = new Frame
            {
                BackgroundColor = Color.FromArgb("#2a2a3e"),
                BorderColor = Color.FromArgb("#3a3a4e"),
                Padding = new Thickness(12, 8),
                CornerRadius = 6,
                Margin = new Thickness(0, 8, 0, 0),
                Content = new Label
                {
                    Text = $"📡 {dongle.DisplayName} ({dongle.Trackers.Count} trackers)",
                    FontAttributes = FontAttributes.Bold,
                    TextColor = Colors.White,
                    FontSize = 14
                }
            };
            trackerListContainer.Children.Add(dongleHeader);

            foreach (var (_, t) in grp)
            {
                var row = new Grid
                {
                    Padding = new Thickness(12, 8),
                    ColumnDefinitions =
                    {
                        new ColumnDefinition { Width = new GridLength(40) },
                        new ColumnDefinition { Width = GridLength.Star },
                        new ColumnDefinition { Width = new GridLength(60) }
                    }
                };

                var rotView = new TrackerRotationView { TrackerRotation = t.Rotation };
                var nameLabel = new Label
                {
                    Text = t.DisplayName,
                    TextColor = Colors.White,
                    VerticalOptions = LayoutOptions.Center
                };
                var infoLabel = new Label
                {
                    Text = $"{t.BatteryLevel:F0}% • {t.Status}",
                    TextColor = Colors.Gray,
                    FontSize = 12,
                    VerticalOptions = LayoutOptions.Center
                };

                var rightStack = new VerticalStackLayout { Spacing = 2 };
                rightStack.Children.Add(nameLabel);
                rightStack.Children.Add(infoLabel);

                row.Add(rotView, 0, 0);
                row.Add(rightStack, 1, 0);
                trackerListContainer.Children.Add(row);
                _trackerRowCache.Add((rotView, nameLabel, infoLabel));
            }
        }
    }

    private void StartButton_Clicked(object? sender, EventArgs e)
    {
        var context = Platform.CurrentActivity ?? Android.App.Application.Context;

        if (!_isTrackerServiceStarted)
        {
            intent = new Intent(context, typeof(TrackerListenerService));
            var endpoint = ipEntry.Text;
            if (IPAddress.TryParse(endpoint ?? "", out _))
            {
                UDPHandler.Endpoint = endpoint!;
                File.WriteAllText(Path.Combine(FileSystem.AppDataDirectory, "config.txt"), endpoint);
                statusLabel.Text = "Starting...";
                statusLabel.TextColor = Colors.LightGreen;
                if (Android.OS.Build.VERSION.SdkInt >= Android.OS.BuildVersionCodes.O)
                    context.StartForegroundService(intent);
                else
                    context.StartService(intent);
                _isTrackerServiceStarted = true;
                startButton.Text = "Stop";
            }
            else
            {
                statusLabel.Text = "Invalid IP address";
                statusLabel.TextColor = Colors.OrangeRed;
            }
        }
        else
        {
            TrackerListenerService.Instance?.StopTrackerWork();
            TrackerListenerService.Instance?.StopSelf();
            try { context?.StopService(intent); } catch { }
            _isTrackerServiceStarted = false;
            startButton.Text = "Start";
            statusLabel.Text = "Stopped";
            statusLabel.TextColor = Colors.Orange;
        }
    }

    private void RefreshButton_Clicked(object? sender, EventArgs e)
    {
        UDPHandler.ForceUDPClientsToDoHandshake();
    }

    private void LoadConfig()
    {
        var configPath = Path.Combine(FileSystem.AppDataDirectory, "config.txt");
        if (File.Exists(configPath))
            ipEntry.Text = File.ReadAllText(configPath);
    }
}
