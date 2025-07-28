using static Everything_To_IMU_SlimeVR.SlimeVR.FirmwareConstants;
namespace EsbImuReceiverToLan.Tracking.Trackers.HID {
    public class AndroidHidDevice {
        private string id;
        private string? firmwareVersion;
        private string? hardwareIdentifier;
        private string name;
        private BoardType boardType;
        private McuType mcuType;
        private Dictionary<int, Tracker> trackers = new();

        public string Id { get => id; set => id = value; }
        public string? FirmwareVersion { get => firmwareVersion; set => firmwareVersion = value; }
        public string? HardwareIdentifier { get => hardwareIdentifier; set => hardwareIdentifier = value; }
        public string Name { get => name; set => name = value; }
        public BoardType BoardType { get => boardType; set => boardType = value; }
        public McuType McuType { get => mcuType; set => mcuType = value; }
        public Dictionary<int, Tracker> Trackers { get => trackers; set => trackers = value; }
    }
}