// Basic namespaces
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using HidSharp;
using HidSharp.Reports;
using System.IO;
using System.Numerics;
using static Everything_To_IMU_SlimeVR.SlimeVR.FirmwareConstants;
using EsbImuReceiverToLAN;
using static SlimeVR.Tracking.Trackers.HID.TrackersHID;
using Everything_To_IMU_SlimeVR.SlimeVR;
using System.Collections;
using System.Text;
using System.Threading.Tasks;
using Everything_To_IMU_SlimeVR.Utility;

// Your namespace here
namespace SlimeVR.Tracking.Trackers.HID {
    public class TrackersHID {
        private const int HID_TRACKER_RECEIVER_VID = 0x1209;
        private const int HID_TRACKER_RECEIVER_PID = 0x7690;
        private const int PACKET_SIZE = 16;

        private readonly List<HIDDevice> devices = new();
        private readonly Dictionary<string, List<int>> devicesBySerial = new();
        private readonly Dictionary<HidDevice, List<int>> devicesByHID = new();
        private readonly Dictionary<HidDevice, int> lastDataByHID = new();

        private readonly HidDeviceLoader hidLoader;

        private readonly Thread dataReadThread;
        private readonly Thread deviceEnumerateThread;
        public event EventHandler<Tracker> trackersConsumer;

        public TrackersHID() {
            hidLoader = new HidDeviceLoader();

            dataReadThread = new Thread(DataRead) {
                IsBackground = true,
                Name = "hidsharp data reader"
            };
            dataReadThread.Start();

            deviceEnumerateThread = new Thread(DeviceEnumerateLoop) {
                IsBackground = true,
                Name = "hidsharp device enumerator"
            };
            deviceEnumerateThread.Start();
        }

        private void CheckConfigureDevice(HidDevice hidDevice) {
            if (hidDevice.VendorID == HID_TRACKER_RECEIVER_VID &&
                hidDevice.ProductID == HID_TRACKER_RECEIVER_PID) {
                if (!hidDevice.TryOpen(out var stream)) {
                    Console.WriteLine($"[TrackerServer] Unable to open device: {hidDevice.DevicePath}");
                    return;
                }

                string serial = hidDevice.GetSerialNumber() ?? "Unknown HID Device";

                if (devicesBySerial.TryGetValue(serial, out var existingList)) {
                    devicesByHID[hidDevice] = existingList;

                    lock (devices) {
                        foreach (int id in existingList) {
                            var device = devices[id];
                            foreach (var tracker in device.Trackers.Values) {
                                if (tracker.Status == TrackerStatus.Disconnected) {
                                    tracker.Status = TrackerStatus.OK;
                                }
                            }
                        }
                    }

                    Console.WriteLine($"[TrackerServer] Linked HID device reattached: {serial}");
                    return;
                }

                var list = new List<int>();
                devicesBySerial[serial] = list;
                devicesByHID[hidDevice] = list;
                lastDataByHID[hidDevice] = 0;

                Console.WriteLine($"[TrackerServer] (Probably) Compatible HID device detected: {serial}");
            }
        }

        private void RemoveDevice(HidDevice device) {
            if (devicesByHID.TryGetValue(device, out var ids)) {
                lock (devices) {
                    foreach (int id in ids) {
                        var dev = devices[id];
                        foreach (var tracker in dev.Trackers.Values) {
                            if (tracker.Status == TrackerStatus.OK) {
                                tracker.Status = TrackerStatus.Disconnected;
                            }
                        }
                    }
                }
                devicesByHID.Remove(device);
                Console.WriteLine($"[TrackerServer] Linked HID device removed: {device.GetSerialNumber()}");
            }
        }

        private void DeviceEnumerateLoop() {
            Thread.Sleep(100); // Delayed start
            while (true) {
                Thread.Sleep(1000);
                DeviceEnumerate();
            }
        }

        private void DeviceEnumerate() {
            var allDevices = hidLoader.GetDevices(HID_TRACKER_RECEIVER_VID, HID_TRACKER_RECEIVER_PID).ToList();

            lock (devicesByHID) {
                var removeList = devicesByHID.Keys.Except(allDevices).ToList();
                foreach (var device in removeList) {
                    RemoveDevice(device);
                }

                foreach (var device in devicesByHID.Keys) {
                    if (lastDataByHID.TryGetValue(device, out int age) && age > 100) {
                        Console.WriteLine($"[TrackerServer] Reopening device {device.GetSerialNumber()} after no data received");
                        device.TryOpen(out _);
                    }
                }

                foreach (var device in allDevices) {
                    if (!devicesByHID.ContainsKey(device)) {
                        CheckConfigureDevice(device);
                    }
                }
            }
        }

        private void DataReadLoop() {
            var dataReceived = new byte[64];
            while (true) {
                Thread.Sleep(1);

                lock (devicesByHID) {
                    foreach (var kvp in devicesByHID) {
                        var device = kvp.Key;
                        var deviceList = kvp.Value;

                        try {
                            if (!device.TryOpen(out var stream)) continue;

                            int bytesRead = stream.Read(dataReceived, 0, dataReceived.Length);
                            if (bytesRead <= 0) continue;

                            if (bytesRead % PACKET_SIZE != 0) {
                                Console.WriteLine("[TrackerServer] Malformed HID packet, ignoring");
                                continue;
                            }

                            lastDataByHID[device] = 0; // reset timer

                            int packetCount = bytesRead / PACKET_SIZE;
                            for (int i = 0; i < packetCount * PACKET_SIZE; i += PACKET_SIZE) {
                                int packetType = dataReceived[i];
                                int id = dataReceived[i + 1];

                                switch (packetType) {
                                    case 0: {
                                            int batt = dataReceived[i + 2];
                                            int batt_v = dataReceived[i + 3];
                                            int temp = dataReceived[i + 4];
                                            int brd_id = dataReceived[i + 5];
                                            int mcu_id = dataReceived[i + 6];
                                            // skipping imu_id (i + 8) and mag_id (i + 9) for now, but can add if needed

                                            // Firmware date is ushort little-endian stored at (i + 10, i + 11)
                                            int fw_date = (dataReceived[i + 11] << 8) | dataReceived[i + 10];
                                            int fw_major = dataReceived[i + 12];
                                            int fw_minor = dataReceived[i + 13];
                                            int fw_patch = dataReceived[i + 14];
                                            int rssi = dataReceived[i + 15];

                                            // Decode firmware date (example logic, based on original)
                                            int firmwareYear = 2020 + ((fw_date >> 9) & 0x7F);
                                            int firmwareMonth = (fw_date >> 5) & 0x0F;
                                            int firmwareDay = fw_date & 0x1F;
                                            string firmwareDate = $"{firmwareYear:D4}-{firmwareMonth:D2}-{firmwareDay:D2}";

                                            Console.WriteLine($"[Packet0] Batt: {batt}, BattV: {batt_v}, Temp: {temp}, BoardID: {brd_id}, MCU: {mcu_id}, FW: {fw_major}.{fw_minor}.{fw_patch} (Build {firmwareDate}), RSSI: {rssi}");
                                            // TODO: Assign info to device/tracker

                                            break;
                                        }

                                    case 1: {
                                            short[] q = new short[4];
                                            short[] a = new short[3];
                                            for (int j = 0; j < 4; j++) {
                                                q[j] = (short)((dataReceived[i + 2 + j * 2 + 1] << 8) | dataReceived[i + 2 + j * 2]);
                                            }
                                            for (int j = 0; j < 3; j++) {
                                                a[j] = (short)((dataReceived[i + 10 + j * 2 + 1] << 8) | dataReceived[i + 10 + j * 2]);
                                            }

                                            Vector3 accel = new Vector3(a[0], a[1], a[2]) * (1.0f / (1 << 7));
                                            Vector4 quatRaw = new Vector4(q[0], q[1], q[2], q[3]) * (1.0f / (1 << 15));
                                            Quaternion rotation = new Quaternion(quatRaw.X, quatRaw.Y, quatRaw.Z, quatRaw.W);

                                            Console.WriteLine($"[Packet1] ID: {id}, Rot: {rotation}, Accel: {accel}");
                                            // TODO: Assign rotation and acceleration to corresponding tracker
                                            break;
                                        }
                                    case 2: {
                                            int batt = dataReceived[i + 2];
                                            int batt_v = dataReceived[i + 3];
                                            int temp = dataReceived[i + 4];

                                            // Decode quantized quaternion (exponential map)
                                            uint q_buf = BitConverter.ToUInt32(dataReceived, i + 5);
                                            float[] v = new float[3];
                                            v[0] = (q_buf & 0x3FF);             // 10 bits
                                            v[1] = (q_buf >> 10) & 0x7FF;       // 11 bits
                                            v[2] = (q_buf >> 21) & 0x7FF;       // 11 bits

                                            v[0] /= (1 << 10);
                                            v[1] /= (1 << 11);
                                            v[2] /= (1 << 11);

                                            for (int j = 0; j < 3; j++) {
                                                v[j] = v[j] * 2f - 1f;
                                            }

                                            float d = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
                                            float invSqrtD = 1f / MathF.Sqrt(d + 1e-6f);
                                            float a = (MathF.PI / 2f) * d * invSqrtD;
                                            float s = MathF.Sin(a);
                                            float k = s * invSqrtD;
                                            Quaternion rot = new Quaternion(k * v[0], k * v[1], k * v[2], MathF.Cos(a));

                                            // Acceleration
                                            short[] a_vals = new short[3];
                                            for (int j = 0; j < 3; j++) {
                                                a_vals[j] = (short)((dataReceived[i + 9 + j * 2 + 1] << 8) | dataReceived[i + 9 + j * 2]);
                                            }
                                            Vector3 accel = new Vector3(a_vals[0], a_vals[1], a_vals[2]) * (1.0f / (1 << 7));

                                            int rssi = dataReceived[i + 15];

                                            Console.WriteLine($"[Packet2] ID: {id}, Rot: {rot}, Accel: {accel}, Batt: {batt}, Temp: {temp}, RSSI: {rssi}");
                                            // TODO: Assign to tracker
                                            break;
                                        }
                                    case 3: {
                                            int svr_status = dataReceived[i + 2];
                                            // int raw_status = dataReceived[i + 3]; // raw tracker status, optional
                                            int rssi = dataReceived[i + 15];

                                            Console.WriteLine($"[Packet3] Server Status: {svr_status}, RSSI: {rssi}");
                                            // TODO: Map svr_status to TrackerStatus enum and assign to tracker.Status
                                            // TODO: Assign signal strength as -rssi

                                            break;
                                        }
                                    case 4: {
                                            short[] q = new short[4];
                                            short[] m = new short[3];

                                            for (int j = 0; j < 4; j++) {
                                                q[j] = (short)((dataReceived[i + 2 + j * 2 + 1] << 8) | dataReceived[i + 2 + j * 2]);
                                            }
                                            for (int j = 0; j < 3; j++) {
                                                m[j] = (short)((dataReceived[i + 10 + j * 2 + 1] << 8) | dataReceived[i + 10 + j * 2]);
                                            }

                                            // Convert to float and scale
                                            float scaleRot = 1f / (1 << 15);
                                            float scaleMag = 1000f / (1 << 10); // milligauss scale

                                            // Quaternion is w,x,y,z (as in original Kotlin)
                                            var rot = new Quaternion(
                                                q[3] * scaleRot,
                                                q[0] * scaleRot,
                                                q[1] * scaleRot,
                                                q[2] * scaleRot
                                            );

                                            var mag = new Vector3(m[0] * scaleMag, m[1] * scaleMag, m[2] * scaleMag);

                                            Console.WriteLine($"[Packet4] Rotation: {rot}, Magnetometer: {mag}");
                                            // TODO: Assign rotation and magnetometer to tracker

                                            break;
                                        }


                                }
                            }
                        } catch (IOException) {
                            continue;
                        }
                    }
                }
            }
        }
        private void SetUpSensor(HIDDevice device, int trackerId, ImuType sensorType, TrackerStatus sensorStatus, MagnetometerStatus magStatus) {
            if (!device.Trackers.TryGetValue(trackerId, out Tracker imuTracker)) {
                string formattedHWID = device.HardwareIdentifier.Replace(":", "").Length > 5
                    ? device.HardwareIdentifier.Replace(":", "").Substring(device.HardwareIdentifier.Replace(":", "").Length - 5)
                    : device.HardwareIdentifier.Replace(":", "");

                imuTracker = new Tracker(device, trackerId, $"{device.Name}/{trackerId}", $"Tracker {formattedHWID}", true, true, true, sensorType, true, true, true, false, magStatus, sensorStatus);

                device.Trackers[trackerId] = imuTracker;

                trackersConsumer?.Invoke(this, imuTracker);

                Console.WriteLine($"[TrackerServer] Added sensor {trackerId} for {device.Name}, type {sensorType}");
            } else {
                imuTracker.Status = sensorStatus;
            }
        }
        private HIDDevice DeviceIdLookup(HidDevice hidDevice, int deviceId, string deviceName, List<int> deviceList) {
            lock (devices) {
                // Try to find existing device by hidId in the deviceList
                foreach (var index in deviceList) {
                    var dev = devices[index];
                    if (dev.Id == deviceId) {
                        return dev;
                    }
                }

                // If deviceName is null, device isn't registered yet
                if (deviceName == null) {
                    return null;
                }

                // Create and register a new HIDDevice
                var newDevice = new HIDDevice(deviceId) {
                    Name = deviceName,
                    Manufacturer = "HID Device",
                    HardwareIdentifier = deviceName
                };

                devices.Add(newDevice);
                deviceList.Add(devices.Count - 1);

                // Example: You might have a VRServer instance managing devices
                DeviceManager.Instance.AddDevice(newDevice);

                Console.WriteLine($"[TrackerServer] Added device {deviceName} for {hidDevice.GetSerialNumber()}, id {deviceId}");

                return newDevice;
            }
        }
        private void DataRead() {
            while (true) {
                Thread.Sleep(1);
                int[] q = new int[4];
                int[] a = new int[3];
                int[] m = new int[3];

                lock (devicesByHID) {
                    bool devicesPresent = false;
                    bool devicesDataReceived = false;

                    foreach (var kvp in devicesByHID) {
                        HidDevice hidDevice = kvp.Key;
                        List<int> deviceList = kvp.Value;

                        try {
                            HidStream stream = null;
                            if (!hidDevice.TryOpen(out stream)) {
                                continue;
                            }
                            byte[] dataReceived = new byte[64];
                          //  //byte[] buffer = new byte[64]; // or the report length expected
                            int bytesRead = stream.Read(dataReceived, 0, dataReceived.Length);
                            // dataReceived = (sbyte[])(Array)buffer;
                            if (bytesRead <= 0)
                                continue;

                            devicesPresent = true;

                            if (dataReceived.Length % PACKET_SIZE != 0) {
                                Console.WriteLine("[TrackerServer] Malformed HID packet, ignoring");
                                continue;
                            }

                            devicesDataReceived = true;
                            lastDataByHID[hidDevice] = 0;

                            int packetCount = dataReceived.Length / PACKET_SIZE;

                            for (int i = 0; i < packetCount * PACKET_SIZE; i += PACKET_SIZE) {
                                int packetType = dataReceived[i] & 0xFF;
                                int id = dataReceived[i + 1] & 0xFF;
                                int trackerId = 0; // no extensions in this context
                                int deviceId = id;

                                if (packetType == 255) // Device register packet
                                {
                                    byte[] data = new byte[8];
                                    Array.Copy(dataReceived, i + 2, data, 0, 8);
                                    ulong addr = BitConverter.ToUInt64(data, 0) & 0xFFFFFFFFFFFF;
                                    string deviceName = addr.ToString("X12");
                                    DeviceIdLookup(hidDevice, deviceId, deviceName, deviceList);
                                    continue;
                                }

                                var device = DeviceIdLookup(hidDevice, deviceId, null, deviceList);
                                if (device == null) {
                                    continue;
                                }

                                if (packetType == 0) // Tracker register
                                {
                                    int imuId = dataReceived[i + 8];
                                    int magId = dataReceived[i + 9];
                                    var sensorType = (ImuType)imuId;
                                    var magStatus = (MagnetometerStatus)magId;
                                    if (sensorType != null && magStatus != null)
                                        SetUpSensor(device, trackerId, sensorType, TrackerStatus.OK, magStatus);
                                }

                                var tracker = device.GetTracker(trackerId);
                                if (tracker == null)
                                    continue;

                                // Variables for data fields
                                int? batt = null, batt_v = null, temp = null, brd_id = null, mcu_id = null;
                                int? fw_date = null, fw_major = null, fw_minor = null, fw_patch = null;
                                int? svr_status = null, rssi = null;

                                switch (packetType) {
                                    case 0: // device info
                                        batt = dataReceived[i + 2];
                                        batt_v = dataReceived[i + 3];
                                        temp = dataReceived[i + 4];
                                        brd_id = dataReceived[i + 5];
                                        mcu_id = dataReceived[i + 6];
                                        fw_date = (dataReceived[i + 11] << 8) | dataReceived[i + 10];
                                        fw_major = dataReceived[i + 12];
                                        fw_minor = dataReceived[i + 13];
                                        fw_patch = dataReceived[i + 14];
                                        rssi = dataReceived[i + 15];
                                        break;

                                    case 1: // full precision quat and accel
                                        for (int j = 0; j < 4; j++) {
                                            q[j] = (dataReceived[i + 2 + j * 2 + 1] << 8) | dataReceived[i + 2 + j * 2];
                                        }
                                        for (int j = 0; j < 3; j++) {
                                            a[j] = (dataReceived[i + 10 + j * 2 + 1] << 8) | dataReceived[i + 10 + j * 2];
                                        }
                                        break;

                                    case 2: // reduced precision quat and accel with data
                                        batt = dataReceived[i + 2];
                                        batt_v = dataReceived[i + 3];
                                        temp = dataReceived[i + 4];
                                        byte[] data = new byte[4];
                                        Array.Copy(dataReceived, i + 5, data, 0, 4);
                                        uint q_buf = BitConverter.ToUInt32((byte[])(Array)data, 0);
                                        q[0] = (int)(q_buf & 1023);
                                        q[1] = (int)((q_buf >> 10) & 2047);
                                        q[2] = (int)((q_buf >> 21) & 2047);
                                        for (int j = 0; j < 3; j++) {
                                            a[j] = (dataReceived[i + 9 + j * 2 + 1] << 8) | dataReceived[i + 9 + j * 2];
                                        }
                                        rssi = dataReceived[i + 15];
                                        break;

                                    case 3: // status
                                        svr_status = dataReceived[i + 2];
                                        rssi = dataReceived[i + 15];
                                        break;

                                    case 4: // full precision quat and mag
                                        for (int j = 0; j < 4; j++) {
                                            q[j] = (dataReceived[i + 2 + j * 2 + 1] << 8) | dataReceived[i + 2 + j * 2];
                                        }
                                        for (int j = 0; j < 3; j++) {
                                            m[j] = (dataReceived[i + 10 + j * 2 + 1] << 8) | dataReceived[i + 10 + j * 2];
                                        }
                                        break;
                                }

                                // Assign battery level
                                if (batt != null) {
                                    tracker.BatteryLevel = (batt == 128) ? 1f : (batt.Value & 127);
                                }
                                // Battery voltage
                                if (batt_v != null) {
                                    tracker.BatteryVoltage = (batt_v.Value + 245f) / 100f;
                                }
                                // Temperature
                                if (temp != null)
                                    tracker.Temperature = (temp > 0) ? (temp.Value / 2f - 39f) : (float?)null;

                                // Board Type
                                if (brd_id != null) {
                                    var boardType = (BoardType)brd_id.Value;
                                    if (boardType != null)
                                        device.BoardType = boardType;
                                }

                                // MCU Type
                                if (mcu_id != null) {
                                    var mcuType = (McuType)mcu_id.Value;
                                    if (mcuType != null)
                                        device.McuType = mcuType;
                                }

                                // Firmware version string
                                if (fw_date != null && fw_major != null && fw_minor != null && fw_patch != null) {
                                    int firmwareYear = 2020 + ((fw_date.Value >> 9) & 127);
                                    int firmwareMonth = (fw_date.Value >> 5) & 15;
                                    int firmwareDay = fw_date.Value & 31;
                                    string firmwareDate = $"{firmwareYear:D4}-{firmwareMonth:D2}-{firmwareDay:D2}";
                                    device.FirmwareVersion = $"{fw_major}.{fw_minor}.{fw_patch} (Build {firmwareDate})";
                                }

                                // Tracker status
                                if (svr_status != null) {
                                    var status = (TrackerStatus)svr_status.Value;
                                    if (status != null)
                                        tracker.Status = status;
                                }

                                // RSSI / Signal strength
                                if (rssi != null)
                                    tracker.SignalStrength = -rssi.Value;

                                // Rotation and acceleration
                                if (packetType == 1 || packetType == 4) {
                                    // Convert Q15 short to float and reorder quaternion as w,x,y,z
                                    var rot = new Quaternion(
                                        q[3] / 32768f,
                                        q[0] / 32768f,
                                        q[1] / 32768f,
                                        q[2] / 32768f
                                    );

                                    // Apply axes offset rotation if needed (not shown here)

                                    tracker.SetRotation(rot);
                                }

                                if (packetType == 2) {
                                    float[] v = new float[3];
                                    v[0] = q[0] / 1024f;
                                    v[1] = q[1] / 2048f;
                                    v[2] = q[2] / 2048f;

                                    for (int x = 0; x < 3; x++)
                                        v[x] = v[x] * 2 - 1;

                                    float d = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
                                    float invSqrtD = 1.0f / (float)Math.Sqrt(d + 1e-6f);
                                    float aAngle = (float)(Math.PI / 2) * d * invSqrtD;
                                    float s = (float)Math.Sin(aAngle);
                                    float k = s * invSqrtD;

                                    var rot = new Quaternion(
                                        (float)Math.Cos(aAngle),
                                        k * v[0],
                                        k * v[1],
                                        k * v[2]
                                    );

                                    // Apply axes offset rotation if needed

                                    tracker.SetRotation(rot);
                                }

                                if (packetType == 1 || packetType == 2) {
                                    Vector3 acceleration = new Vector3(a[0], a[1], a[2]) * (1f / 128f);
                                    tracker.SetAcceleration(acceleration);
                                }

                                if (packetType == 4) {
                                    Vector3 magnetometer = new Vector3(m[0], m[1], m[2]) * (1000f / 1024f);
                                    tracker.SetMagVector(magnetometer);
                                }

                                if (packetType == 1 || packetType == 2 || packetType == 4) {
                                    tracker.DataTick();
                                }
                            }
                        } catch (IOException) {
                            continue;
                        }
                    }

                    if (!devicesPresent)
                        Thread.Sleep(10);
                    else if (!devicesDataReceived)
                        Thread.Sleep(1);
                }
            }
        }


        // Placeholder classes for integration:
        public class HIDDevice {
            public int Id;
            public string Name;
            public string Manufacturer;
            public string HardwareIdentifier;
            public Dictionary<int, Tracker> Trackers = new();
            public HIDDevice(int id) { Id = id; }

            public BoardType BoardType { get; internal set; }
            public McuType McuType { get; internal set; }
            public string FirmwareVersion { get; internal set; }
            public Tracker? GetTracker(int id) {
                if (Trackers.TryGetValue(id, out var tracker)) {
                    return tracker;
                }
                return null;
            }
        }

        public class Tracker {

            public HIDDevice Device { get; internal set; }
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
                get { return _batteryLevel; }
                internal set {
                    _udpHandler.SetSensorBattery(value);
                    _batteryLevel = value;
                }
            }
            public float BatteryVoltage { get; internal set; }
            public float? Temperature { get; internal set; }
            public int SignalStrength { get; internal set; }

            public TrackerStatus Status = TrackerStatus.Disconnected;
            UDPHandler _udpHandler;
            private float _batteryLevel;

            public Tracker(HIDDevice device, int trackerNum, string name, string displayName, bool hasRotation, bool hasAcceleration, bool userEditable, ImuType imuType, bool allowFiltering, bool needsReset, bool needsMounting, bool usesTimeout, MagnetometerStatus magStatus, TrackerStatus status) {
                Device = device;
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
                _udpHandler = new UDPHandler(device.FirmwareVersion + "_UDP", Encoding.UTF8.GetBytes("UDP_" + device.HardwareIdentifier), device.BoardType, ImuType, device.McuType, 1);
            }

            public void SetRotation(Quaternion q) {
                _udpHandler.SetSensorRotation(q, 0);
            }
            public void SetAcceleration(Vector3 a) {
                _udpHandler.SetSensorAcceleration(a, 0);
            }
            public void SetMagVector(Vector3 m) {
                _udpHandler.SetSensorMagnetometer(m, 0);
            }
            public async Task DataTick() {

            }
        }

        public enum TrackerStatus {
            OK,
            Disconnected
        }
    }
}