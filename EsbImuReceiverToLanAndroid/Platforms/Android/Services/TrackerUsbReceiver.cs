using Android.App;
using Android.Content;
using Android.Runtime;
using EsbReceiverToLanAndroid.Platforms.Android.Services;
using AOS = Android.OS;

namespace EsbReceiverToLanAndroid { 
[BroadcastReceiver(Enabled = true, Exported = true, Name = "com.SebaneStudios.EsbReceiverToLanAndroid.TrackerUsbReceiver")]
    [IntentFilter(new[] { "android.hardware.usb.action.USB_DEVICE_ATTACHED" })]
    [Preserve(AllMembers = true)]
    public class TrackerUsbReceiver : BroadcastReceiver {
        public override void OnReceive(Context context, Intent intent) {
            var serviceIntent = new Intent(context, typeof(TrackerListenerService));
            serviceIntent.SetPackage(context.PackageName); // ✅ MAKE IT EXPLICIT

            if (AOS.Build.VERSION.SdkInt >= AOS.BuildVersionCodes.O)
                context.StartForegroundService(serviceIntent);
            else
                context.StartService(serviceIntent);
        }

    }
}