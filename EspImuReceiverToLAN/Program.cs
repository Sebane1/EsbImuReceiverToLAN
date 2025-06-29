using Everything_To_IMU_SlimeVR.Utility;
using SlimeVR.Tracking.Trackers.HID;

namespace EsbImuReceiverToLAN {
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
