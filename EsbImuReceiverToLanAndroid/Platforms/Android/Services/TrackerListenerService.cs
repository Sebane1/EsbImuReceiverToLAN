using Android;
using Android.App;
using Android.Content;
using Android.Content.PM;
using Android.Hardware.Usb;
using Android.OS;
using Android.Runtime;
using Android.Util;
using AndroidX.Core.App;
using AndroidX.Core.Content;
using EsbImuReceiverToLan.Tracking.Trackers.HID;
using EsbReceiverToLanAndroid;
using EsbReceiverToLanAndroid.Models;
using SlimeImuProtocol.SlimeVR;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using static Android.Manifest;
using Resource = Microsoft.Maui.Controls.Resource;

namespace EsbReceiverToLanAndroid.Platforms.Android.Services;

[Preserve(AllMembers = true)]
[Service(ForegroundServiceType = ForegroundService.TypeDataSync, Exported = true)]
public class TrackerListenerService : Service {
    private static bool _running = false;
    private Thread _thread;
    private static TrackerListenerService _instance;
    private UsbDevice? _pendingUsbDevice;

    public static TrackerListenerService Instance { get => _instance; set => _instance = value; }
    TrackersHID_Android _trackersHid;
    public override void OnCreate() {
        base.OnCreate();
        Log.Info("TrackerListenerService", "Service Created");
        _instance = this;
        StartTrackerWork();
        ShowNotification();
    }


    private void StartTrackerWork() {
        if (_running) return; // Already running, ignore duplicate start

        // Check if IP is configured
        if (string.IsNullOrWhiteSpace(UDPHandler.Endpoint) || !IPAddress.TryParse(UDPHandler.Endpoint, out _)) {
            Log.Warn("TrackerListenerService", "Cannot start: No valid UDP endpoint configured.");
            // Optionally stop the service immediately
            StopSelf();
            return;
        }

        _running = true;
        _thread = new Thread(HIDTrackerReader);
        _thread.Start();
    }
    public void StopTrackerWork() {
        Log.Info("TrackerListenerService", "Stopping tracker work...");

        _trackersHid?.StopReading();
        _trackersHid = null;

        if (_thread != null && _thread.IsAlive) {
            try {
                _thread?.Interrupt(); // Soft cancel
            } catch (ThreadInterruptedException) {
                Log.Warn("TrackerListenerService", "Thread interrupted during shutdown.");
            }
        }
        _thread = null;
        _running = false;
    }



    private void HIDTrackerReader() {
        if(_trackersHid != null) {
            _trackersHid?.StopReading();
        }
        Thread.Sleep(300);
        var deviceFromIntent = _instance?._pendingUsbDevice;
        if (_instance != null)
            _instance._pendingUsbDevice = null;
        _trackersHid = new TrackersHID_Android(deviceFromIntent);
    }

    public TrackerSnapshot? GetTrackerSnapshot() => _trackersHid?.GetTrackerSnapshot();

    public void Vibrate() {
        var vibrator = (Vibrator)GetSystemService(VibratorService);
        if (Build.VERSION.SdkInt >= BuildVersionCodes.O) {
            vibrator.Vibrate(VibrationEffect.CreateOneShot(300, VibrationEffect.DefaultAmplitude));
        } else {
#pragma warning disable CS0618
            vibrator.Vibrate(300);
#pragma warning restore CS0618
        }
    }

    public override IBinder OnBind(Intent intent) => null;

    public override void OnDestroy() {
        Log.Info("TrackerListenerService", "Service Destroyed");
        StopTrackerWork();
        UDPHandler.ForceDestroy();
        base.OnDestroy();
    }

    private void ShowNotification() {
        var notificationManager = (NotificationManager)GetSystemService(NotificationService);
        if (Build.VERSION.SdkInt >= BuildVersionCodes.O) {
            var channel = new NotificationChannel("tracker_listener_channel", "Esb Tracker Listener", NotificationImportance.Default);
            notificationManager.CreateNotificationChannel(channel);
        }

        // Intent to launch your main activity (adjust your MainActivity class path)
        Intent intent = new Intent(this, typeof(MainActivity));
        intent.SetFlags(ActivityFlags.ClearTop | ActivityFlags.SingleTop);

        // Create immutable PendingIntent for Android 14+
        var pendingIntent = PendingIntent.GetActivity(this, 0, intent, PendingIntentFlags.Immutable);

        Notification.Builder builder;
        if (Build.VERSION.SdkInt >= BuildVersionCodes.O) {
            builder = new Notification.Builder(this, "tracker_listener_channel");
        } else {
            builder = new Notification.Builder(this);
        }

        var notification = builder
            .SetContentTitle("Esb Tracker Listener")
            .SetContentText("Converting Esb Tracker Packets")
            .SetSmallIcon(Resource.Drawable.dotnet_bot)
            .SetContentIntent(pendingIntent)  // <-- set PendingIntent here
            .SetOngoing(true)                 // keeps the notification persistent
            .Build();

        StartForeground(1, notification);
    }

    public override StartCommandResult OnStartCommand(Intent intent, StartCommandFlags flags, int startId) {
        string action = intent?.Action;

        if (action == "com.SebaneStudios.EsbReceiverToLanAndroid.ACTION_USB_DEVICE_ATTACHED") {
            var device = intent?.GetParcelableExtra(UsbManager.ExtraDevice) as UsbDevice;
            if (device != null && device.VendorId == 0x1209 && device.ProductId == 0x7690) {
                if (_trackersHid != null) {
                    _trackersHid.TryOpenDeviceFromIntent(device);
                } else {
                    _pendingUsbDevice = device;
                }
            }
            return StartCommandResult.Sticky;
        }

        if (action == "com.SebaneStudios.EsbReceiverToLanAndroid.ACTION_USB_DEVICE_DETACHED") {
            var device = intent?.GetParcelableExtra(UsbManager.ExtraDevice) as UsbDevice;
            if (device != null) {
                string deviceKey = device.DeviceName ?? device.DeviceId.ToString();
                _trackersHid?.RemoveDevice(deviceKey);
                Log.Info("TrackerListenerService", $"USB device detached: {deviceKey}");
                if (_trackersHid?.OpenDeviceCount == 0) {
                    SendBroadcast(new Intent(TrackerUsbReceiver.ActionLastDeviceDetached).SetPackage(PackageName));
                    StopTrackerWork();
                    StopSelf();
                    return StartCommandResult.NotSticky;
                }
            }
            return StartCommandResult.NotSticky;
        }

        return StartCommandResult.Sticky;
    }
}
