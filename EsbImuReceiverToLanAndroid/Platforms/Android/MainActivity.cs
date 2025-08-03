using Android.App;
using Android.Content;
using Android.Content.PM;
using Android.OS;
using EsbReceiverToLanAndroid.Platforms.Android.Services;

namespace EsbReceiverToLanAndroid
{

    [Activity(Theme = "@style/Maui.SplashTheme", MainLauncher = true, LaunchMode = LaunchMode.SingleTop, ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation | ConfigChanges.UiMode | ConfigChanges.ScreenLayout | ConfigChanges.SmallestScreenSize | ConfigChanges.Density)]
    public class MainActivity : MauiAppCompatActivity
    {
        protected override void OnDestroy() {
            base.OnDestroy();
            StopService(new Intent(this, typeof(TrackerListenerService)));
        }

    }
}
