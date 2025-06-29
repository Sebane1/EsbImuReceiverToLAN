using Everything_To_IMU_SlimeVR.Utility;
using EsbImuReceiverToLan.Tracking.Trackers.HID;

namespace EspImuReceiverToLAN {
    internal class Program {
        static TrackersHID _trackersHid;
        static void Main(string[] args) {
            //BigEndianExtensions.SkipCorrection = true;
            _trackersHid = new TrackersHID();
            while (true) {
                Thread.Sleep(10000);
            }
        }
    }
}
