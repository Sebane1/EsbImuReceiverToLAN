using Android.App;
using Android.Content;
using Android.Content.PM;
using Android.Hardware.Usb;
using Android.OS;
using EsbReceiverToLanAndroid.Platforms.Android.Services;
using Microsoft.Maui.Storage;
using SlimeImuProtocol.SlimeVR;

namespace EsbReceiverToLanAndroid
{
    [Activity(Theme = "@style/Maui.SplashTheme", MainLauncher = true, LaunchMode = LaunchMode.SingleTop, ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation | ConfigChanges.UiMode | ConfigChanges.ScreenLayout | ConfigChanges.SmallestScreenSize | ConfigChanges.Density)]
    [IntentFilter(new[] { UsbManager.ActionUsbDeviceAttached })]
    [MetaData(UsbManager.ActionUsbDeviceAttached, Resource = "@xml/device_filter")]
    public class MainActivity : MauiAppCompatActivity
    {
        protected override void OnCreate(Bundle? savedInstanceState)
        {
            base.OnCreate(savedInstanceState);
            HandleUsbDeviceIntent(Intent);
        }

        protected override void OnNewIntent(Intent? intent)
        {
            base.OnNewIntent(intent);
            if (intent != null)
                HandleUsbDeviceIntent(intent);
        }

        private void HandleUsbDeviceIntent(Intent? intent)
        {
            if (intent?.Action != UsbManager.ActionUsbDeviceAttached)
                return;

            var device = intent.GetParcelableExtra(UsbManager.ExtraDevice) as UsbDevice;
            if (device != null)
            {
                LoadConfigAndStartService(device);
            }
        }

        private void LoadConfigAndStartService(UsbDevice? device = null)
        {
            var configPath = Path.Combine(FileSystem.AppDataDirectory, "config.txt");
            if (File.Exists(configPath))
            {
                var savedEndpoint = File.ReadAllText(configPath).Trim();
                if (!string.IsNullOrEmpty(savedEndpoint) && System.Net.IPAddress.TryParse(savedEndpoint, out _))
                    UDPHandler.Endpoint = savedEndpoint;
            }

            TrackerUsbReceiver.OnDeviceConnected?.Invoke(this, EventArgs.Empty);
            var serviceIntent = new Intent(this, typeof(TrackerListenerService));
            serviceIntent.SetPackage(PackageName);
            serviceIntent.SetAction("com.SebaneStudios.EsbReceiverToLanAndroid.ACTION_USB_DEVICE_ATTACHED");
            if (device != null)
                serviceIntent.PutExtra(UsbManager.ExtraDevice, device);
            if (Build.VERSION.SdkInt >= BuildVersionCodes.O)
                StartForegroundService(serviceIntent);
            else
                StartService(serviceIntent);
        }

        protected override void OnDestroy()
        {
            base.OnDestroy();
            StopService(new Intent(this, typeof(TrackerListenerService)));
        }
    }
}
