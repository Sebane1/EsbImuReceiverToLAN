using Android.App;
using Android.Content;
using Android.Hardware.Usb;
using Android.Runtime;
using EsbReceiverToLanAndroid.Platforms.Android.Services;
using AOS = Android.OS;

namespace EsbReceiverToLanAndroid {
    [BroadcastReceiver(Enabled = true, Exported = true, Name = "com.SebaneStudios.EsbReceiverToLanAndroid.TrackerUsbReceiver")]
    [IntentFilter(new[] { "android.hardware.usb.action.USB_DEVICE_ATTACHED" })]
    [IntentFilter(new[] { UsbManager.ActionUsbDeviceDetached })]
    [Preserve(AllMembers = true)]

    public class TrackerUsbReceiver : BroadcastReceiver {
        bool justDisconnected = false;
        bool justConnected = false;
        public static EventHandler OnDeviceConnected;
        public static EventHandler OnDeviceDisconnected;
        Intent serviceIntent = null;
        public override void OnReceive(Context context, Intent intent) {
            string action = intent.Action;

            if (UsbManager.ActionUsbDeviceAttached.Equals(action)) {
                if (!justConnected)
                {
                    OnDeviceConnected?.Invoke(this, EventArgs.Empty);
                    justConnected = true;
                    justDisconnected = false;
                    serviceIntent = new Intent(context, typeof(TrackerListenerService));
                    serviceIntent.SetPackage(context.PackageName);

                    if (AOS.Build.VERSION.SdkInt >= AOS.BuildVersionCodes.O)
                    {
                        context.StartForegroundService(serviceIntent);
                    }
                    else
                    {
                        context.StartService(serviceIntent);
                    }
                }
            } else if (UsbManager.ActionUsbDeviceDetached.Equals(action)) {
                if (!justDisconnected) {
                    OnDeviceDisconnected?.Invoke(this, EventArgs.Empty);
                    justConnected = false;
                    justDisconnected = true;
                    UsbDevice device = (UsbDevice)intent.GetParcelableExtra(UsbManager.ExtraDevice);

                    TrackerListenerService.Instance?.StopTrackerWork();
                    TrackerListenerService.Instance?.StopSelf();
                    if (serviceIntent != null) {
                        try {
                            context?.StopService(serviceIntent);
                        } catch {
                        }
                    }
                }
            }
        }
    }
}