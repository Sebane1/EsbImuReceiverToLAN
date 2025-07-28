using Everything_To_IMU_SlimeVR.SlimeVR;
using System.Numerics;
using System.Text;
using static Everything_To_IMU_SlimeVR.SlimeVR.FirmwareConstants;

namespace EsbImuReceiverToLan.Tracking.Trackers.HID {
    public class Tracker {
        public int TrackerNum { get; internal set; }
        public string Name { get; internal set; }
        public string DisplayName { get; internal set; }
        public bool HasRotation { get; internal set; }
        public bool HasAcceleration { get; internal set; }
        public bool UserEditable { get; internal set; }
        public ImuType ImuType { get; internal set; }
        public bool AllowFiltering { get; internal set; }
        public bool NeedsReset { get; internal set; }
        public bool NeedsMounting { get; internal set; }
        public bool UsesTimeout { get; internal set; }
        public MagnetometerStatus MagStatus { get; internal set; }
        public float BatteryLevel {
            get => _batteryLevel;
            internal set {
                _batteryLevel = value;
                if (_ready)
                    _udpHandler.SetSensorBattery(_batteryLevel, BatteryVoltage);
            }
        }
        public float BatteryVoltage { get; internal set; }
        public float? Temperature { get; internal set; }
        public int SignalStrength { get; internal set; }

        public TrackerStatus Status = TrackerStatus.Disconnected;
        public bool Disconnected { get; set; }

        private UDPHandler _udpHandler;
        private bool _ready;
        private float _batteryLevel;

        // These can be set after construction when device data is parsed
        public string FirmwareVersion { get; set; }
        public string HardwareIdentifier { get; set; }
        public BoardType BoardType { get; set; }
        public McuType McuType { get; set; }

        public Tracker(AndroidHidDevice device, int trackerNum, string name, string displayName, bool hasRotation, bool hasAcceleration,
            bool userEditable, ImuType imuType, bool allowFiltering, bool needsReset,
            bool needsMounting, bool usesTimeout, MagnetometerStatus magStatus) {
            TrackerNum = trackerNum;
            Name = name;
            DisplayName = displayName;
            HasRotation = hasRotation;
            HasAcceleration = hasAcceleration;
            UserEditable = userEditable;
            ImuType = imuType;
            AllowFiltering = allowFiltering;
            NeedsReset = needsReset;
            NeedsMounting = needsMounting;
            UsesTimeout = usesTimeout;
            MagStatus = magStatus;
            Task.Run(() => {
                while (device.FirmwareVersion == null) {
                    Thread.Sleep(1000);
                }
                _udpHandler = new UDPHandler(device.FirmwareVersion + "_EsbToLan", Encoding.UTF8.GetBytes(device.HardwareIdentifier), device.BoardType, ImuType, device.McuType, 1);
                _ready = true;
            });
        }

        public void TryInitialize() {
            if (FirmwareVersion != null && HardwareIdentifier != null) {
                _udpHandler = new UDPHandler(
                    FirmwareVersion + "_EsbToLan",
                    Encoding.UTF8.GetBytes(HardwareIdentifier),
                    BoardType,
                    ImuType,
                    McuType,
                    1
                );
                _ready = true;
            }
        }

        public void SetRotation(Quaternion q) {
            if (_ready)
                _udpHandler.SetSensorRotation(q, 0);
        }

        public void SetAcceleration(Vector3 a) {
            if (_ready)
                _udpHandler.SetSensorAcceleration(a, 0);
        }

        public void SetMagVector(Vector3 m) {
            if (_ready)
                _udpHandler.SetSensorMagnetometer(m, 0);
        }

        public async Task DataTick() {
            // Optional: add update logic if needed.
        }
    }

    public enum TrackerStatus {
        OK,
        Disconnected
    }
}
