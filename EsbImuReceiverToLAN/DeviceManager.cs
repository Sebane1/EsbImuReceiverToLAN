using EsbImuReceiverToLan.Tracking.Trackers.HID;

namespace EspImuReceiverToLAN {
    public class DeviceManager {
        private static DeviceManager _instance = new DeviceManager();
        Dictionary<string, TrackersHID.HIDDevice> _devices = new Dictionary<string, TrackersHID.HIDDevice>();
        private int _nextLocalTrackerId;

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

        public void AddDevice(TrackersHID.HIDDevice newDevice) {
            _devices[newDevice.HardwareIdentifier] = newDevice;
        }
        public int GetNextLocalTrackerId() {
            return ++_nextLocalTrackerId;
        }
    }
}
