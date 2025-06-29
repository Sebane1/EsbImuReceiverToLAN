using SlimeVR.Tracking.Trackers.HID;

namespace EsbImuReceiverToLAN {
    public class DeviceManager {
        private static DeviceManager _instance = new DeviceManager();
        Dictionary<string, TrackersHID.HIDDevice> _devices = new Dictionary<string, TrackersHID.HIDDevice>();

        public static DeviceManager Instance {
            get {
                if (_instance == null) {
                    _instance = new DeviceManager();
                }
                return _instance;
            }
        }
        public DeviceManager() {
            _instance = this;
        }

        internal void AddDevice(TrackersHID.HIDDevice newDevice) {
            _devices[newDevice.HardwareIdentifier] = newDevice;
        }
    }
}
