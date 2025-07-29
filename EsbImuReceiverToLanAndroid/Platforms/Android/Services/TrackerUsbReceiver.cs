using Android.App;
using Android.Content;
using Android.Hardware.Usb;
using Android.Runtime;
using EsbReceiverToLanAndroid.Platforms.Android.Services;
using AOS = Android.OS;

namespace EsbReceiverToLanAndroid { 
[BroadcastReceiver(Enabled = true, Exported = true, Name = "com.SebaneStudios.EsbReceiverToLanAndroid.TrackerUsbReceiver")]
    [IntentFilter(new[] { "android.hardware.usb.action.USB_DEVICE_ATTACHED" })]
    [IntentFilter(new[] {UsbManager.ActionUsbDeviceDetached})]
    [Preserve(AllMembers = true)]
    public class TrackerUsbReceiver : BroadcastReceiver {
        public override void OnReceive(Context context, Intent intent) {
            string action = intent.Action;

            if (UsbManager.ActionUsbDeviceAttached.Equals(action)) {
                var serviceIntent = new Intent(context, typeof(TrackerListenerService));
                serviceIntent.SetPackage(context.PackageName);

                if (AOS.Build.VERSION.SdkInt >= AOS.BuildVersionCodes.O)
                    context.StartForegroundService(serviceIntent);
                else
                    context.StartService(serviceIntent);
            } else if (UsbManager.ActionUsbDeviceDetached.Equals(action)) {
                UsbDevice device = (UsbDevice)intent.GetParcelableExtra(UsbManager.ExtraDevice);

                // Let your service know to stop or clean up this specific device
                var stopIntent = new Intent(context, typeof(TrackerListenerService));
                stopIntent.SetAction("com.SebaneStudios.EsbReceiverToLanAndroid.ACTION_USB_REMOVED");

                context.StartService(stopIntent);
            }
        }

    }
}