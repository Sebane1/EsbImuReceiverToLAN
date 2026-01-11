// Based on https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/desktop/src/main/java/dev/slimevr/desktop/tracking/trackers/hid/TrackersHID.kt
using HidSharp;
using System.Numerics;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;
using EspImuReceiverToLAN;
using SlimeImuProtocol.SlimeVR;
using System.Text;
using SlimeImuProtocol.Utility;
using SlimeImuProtocol;

namespace EsbImuReceiverToLan.Tracking.Trackers.HID
{
    public class TrackersHID
    {
        private const int HID_TRACKER_RECEIVER_VID = 0x1209;
        private const int HID_TRACKER_RECEIVER_PID = 0x7690;
        private const int PACKET_SIZE = 16;

        private readonly List<TrackerDevice> devices = new();
        private readonly Dictionary<string, List<int>> devicesBySerial = new();
        private readonly Dictionary<HidDevice, List<int>> devicesByHID = new();
        private readonly Dictionary<HidDevice, int> lastDataByHID = new();

        private readonly HidDeviceLoader hidLoader;
        private readonly FunctionSequenceManager _functionSequenceManager;
        private readonly Thread dataReadThread;
        private readonly Thread deviceEnumerateThread;

        public Quaternion AXES_OFFSET { get; internal set; }

        public event EventHandler<Tracker> trackersConsumer;

        public TrackersHID()
        {
            hidLoader = new HidDeviceLoader();

            _functionSequenceManager = new FunctionSequenceManager();

            dataReadThread = new Thread(DataRead)
            {
                IsBackground = true,
                Name = "hidsharp data reader"
            };
            dataReadThread.Start();

            deviceEnumerateThread = new Thread(DeviceEnumerateLoop)
            {
                IsBackground = true,
                Name = "hidsharp device enumerator"
            };
            deviceEnumerateThread.Start();

            // Apply AXES_OFFSET * rot
            float angle = -MathF.PI / 2;

            // Create quaternion from axis-angle
            AXES_OFFSET = Quaternion.CreateFromAxisAngle(Vector3.UnitX, angle);
        }

        private void CheckConfigureDevice(HidDevice hidDevice)
        {
            if (hidDevice.VendorID == HID_TRACKER_RECEIVER_VID &&
                hidDevice.ProductID == HID_TRACKER_RECEIVER_PID)
            {
                if (!hidDevice.TryOpen(out var stream))
                {
                    Console.WriteLine($"[TrackerServer] Unable to open device: {hidDevice.DevicePath}");
                    return;
                }

                string serial = hidDevice.GetSerialNumber() ?? "Unknown HID Device";

                if (devicesBySerial.TryGetValue(serial, out var existingList))
                {
                    devicesByHID[hidDevice] = existingList;

                    lock (devices)
                    {
                        foreach (int id in existingList)
                        {
                            var device = devices[id];
                            foreach (var tracker in device.Trackers.Values)
                            {
                                if (tracker.Status == TrackerStatus.Disconnected)
                                {
                                    tracker.Status = TrackerStatus.OK;
                                }
                            }
                        }
                    }

                    Console.WriteLine($"[TrackerServer] Linked HID device reattached: {serial}");

                    UDPHandler.ForceUDPClientsToDoHandshake();
                    return;
                }

                var list = new List<int>();
                devicesBySerial[serial] = list;
                devicesByHID[hidDevice] = list;
                lastDataByHID[hidDevice] = 0;

                Console.WriteLine($"[TrackerServer] (Probably) Compatible HID device detected: {serial}");
            }
        }

        private void DeviceEnumerateLoop()
        {
            Thread.Sleep(100); // Delayed start
            while (true)
            {
                Thread.Sleep(1000);
                DeviceEnumerate();
            }
        }

        private void DeviceEnumerate()
        {
            var allDevices = hidLoader.GetDevices(HID_TRACKER_RECEIVER_VID, HID_TRACKER_RECEIVER_PID).ToList();

            lock (devicesByHID)
            {
                var removeList = devicesByHID.Keys.Except(allDevices).ToList();
                foreach (var device in removeList)
                {
                    RemoveDevice(device);
                }

                foreach (var device in devicesByHID.Keys)
                {
                    if (lastDataByHID.TryGetValue(device, out int age) && age > 100)
                    {
                        Console.WriteLine($"[TrackerServer] Reopening device {device.GetSerialNumber()} after no data received");
                        device.TryOpen(out _);
                    }
                }

                foreach (var device in allDevices)
                {
                    if (!devicesByHID.ContainsKey(device))
                    {
                        CheckConfigureDevice(device);
                    }
                }
            }
        }
        private void RemoveDevice(HidDevice device)
        {
            if (devicesByHID.TryGetValue(device, out var ids))
            {
                lock (devices)
                {
                    foreach (int id in ids)
                    {
                        var dev = devices[id];
                        foreach (var tracker in dev.Trackers.Values)
                        {
                            if (tracker.Status == TrackerStatus.OK)
                            {
                                tracker.Status = TrackerStatus.Disconnected;
                            }
                        }
                    }
                }
                devicesByHID.Remove(device);
                Console.WriteLine($"[TrackerServer] Linked HID device removed: {device.GetSerialNumber()}");
            }
        }
        private void SetUpSensor(TrackerDevice device, int trackerId, ImuType sensorType, TrackerStatus sensorStatus, MagnetometerStatus magStatus)
        {
            if (!device.Trackers.TryGetValue(trackerId, out Tracker imuTracker))
            {
                string formattedHWID = device.HardwareIdentifier.Replace(":", "").Length > 5
                    ? device.HardwareIdentifier.Replace(":", "").Substring(device.HardwareIdentifier.Replace(":", "").Length - 5)
                    : device.HardwareIdentifier.Replace(":", "");

                imuTracker = new Tracker(device,
                    trackerId,
                    $"{device.Name}/{trackerId}",
                    $"Tracker {formattedHWID}",
                    true,
                    true,
                    true,
                    sensorType,
                    true,
                    true,
                    true,
                    false,
                    magStatus,
                    _functionSequenceManager);

                device.Trackers[trackerId] = imuTracker;

                trackersConsumer?.Invoke(this, imuTracker);

                Console.WriteLine($"[TrackerServer] Added sensor {trackerId} for {device.Name}, type {sensorType}");
            }
            else
            {
                imuTracker.Status = sensorStatus;
            }
        }
        private TrackerDevice DeviceIdLookup(HidDevice hidDevice, int deviceId, string deviceName, List<int> deviceList)
        {
            lock (devices)
            {
                // Try to find existing device by hidId in the deviceList
                foreach (var index in deviceList)
                {
                    var dev = devices[index];
                    if (dev.Id == deviceId)
                    {
                        return dev;
                    }
                }

                // If deviceName is null, device isn't registered yet
                if (deviceName == null)
                {
                    return null;
                }

                // Create and register a new HIDDevice
                var device = new TrackerDevice(deviceId)
                {
                    Name = deviceName,
                    Manufacturer = "HID Device",
                    HardwareIdentifier = deviceName
                };

                devices.Add(device);
                deviceList.Add(devices.Count - 1);

                // Example: You might have a VRServer instance managing devices
                DeviceManager.Instance.AddDevice(device);

                Console.WriteLine($"[TrackerServer] Added device {deviceName} for {hidDevice.GetSerialNumber()}, id {deviceId}");

                return device;
            }
        }
        private void DataRead()
        {
            while (true)
            {
                try
                {
                    Thread.Sleep(1);
                    int[] q = new int[4];
                    int[] a = new int[3];
                    int[] m = new int[3];

                    lock (devicesByHID)
                    {
                        bool devicesPresent = false;
                        bool devicesDataReceived = false;

                        foreach (var kvp in devicesByHID)
                        {
                            HidDevice hidDevice = kvp.Key;
                            List<int> deviceList = kvp.Value;

                            HidStream stream = null;
                            if (!hidDevice.TryOpen(out stream))
                            {
                                continue;
                            }
                            byte[] newData = new byte[65]; // 1 byte report ID + 64 bytes payload
                            int bytesRead = stream.Read(newData, 0, newData.Length);
                            int offset = (newData.Length % PACKET_SIZE == 0) ? 0 : 1;
                            int validLength = newData.Length - offset;
                            byte[] dataReceived = new byte[validLength];
                            Array.Copy(newData, offset, dataReceived, 0, validLength);
                            if (dataReceived.Length == 0)
                            {
                                continue;
                            }
                            devicesPresent = true;

                            if (dataReceived.Length % PACKET_SIZE != 0)
                            {
                                Console.WriteLine("[TrackerServer] Malformed HID packet, ignoring");
                                continue;
                            }

                            devicesDataReceived = true;
                            lastDataByHID[hidDevice] = 0;

                            int packetCount = dataReceived.Length / PACKET_SIZE;

                            for (int i = 0; i < packetCount * PACKET_SIZE; i += PACKET_SIZE)
                            {
                                int packetType = dataReceived[i];
                                int id = dataReceived[i + 1];
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
                                if (device == null)
                                {
                                    continue;
                                }

                                if (packetType == 0) // Tracker register
                                {
                                    uint imuId = dataReceived[i + 8];
                                    uint magId = dataReceived[i + 9];
                                    var sensorType = (ImuType)imuId;
                                    var magStatus = (MagnetometerStatus)magId;
                                    if (sensorType != ImuType.UNKNOWN && magStatus != null)
                                    {
                                        SetUpSensor(device, trackerId, sensorType, TrackerStatus.OK, magStatus);
                                    }
                                }

                                var tracker = device.GetTracker(trackerId);
                                if (tracker == null)
                                {
                                    continue;
                                }

                                // Variables for data fields
                                int? batt = null, batt_v = null, temp = null, brd_id = null, mcu_id = null;
                                int? fw_date = null, fw_major = null, fw_minor = null, fw_patch = null;
                                int? svr_status = null, rssi = null;

                                switch (packetType)
                                {
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
                                        for (int j = 0; j < 4; j++)
                                        {
                                            q[j] = (short)((dataReceived[i + 2 + j * 2 + 1]) << 8) | (dataReceived[i + 2 + j * 2]);
                                        }
                                        for (int j = 0; j < 3; j++)
                                        {
                                            a[j] = (short)((dataReceived[i + 10 + j * 2 + 1]) << 8) | (dataReceived[i + 10 + j * 2]);
                                        }
                                        break;

                                    case 2: // reduced precision quat and accel with data
                                        batt = dataReceived[i + 2];
                                        batt_v = dataReceived[i + 3];
                                        temp = dataReceived[i + 4];
                                        byte[] data = new byte[4];
                                        Array.Copy(dataReceived, i + 5, data, 0, 4);
                                        uint q_buf = BitConverter.ToUInt32(data, 0);
                                        q[0] = (int)(q_buf & 1023);
                                        q[1] = (int)((q_buf >> 10) & 2047);
                                        q[2] = (int)((q_buf >> 21) & 2047);
                                        for (int j = 0; j < 3; j++)
                                        {
                                            a[j] = (short)((dataReceived[i + 9 + j * 2 + 1]) << 8) | (dataReceived[i + 9 + j * 2]);
                                        }
                                        rssi = dataReceived[i + 15];
                                        break;

                                    case 3: // status
                                        svr_status = dataReceived[i + 2];
                                        rssi = dataReceived[i + 15];
                                        break;

                                    case 4: // full precision quat and mag
                                        for (int j = 0; j < 4; j++)
                                        {
                                            q[j] = (short)((dataReceived[i + 2 + j * 2 + 1]) << 8) | (dataReceived[i + 2 + j * 2]);
                                        }
                                        for (int j = 0; j < 3; j++)
                                        {
                                            m[j] = (short)((dataReceived[i + 10 + j * 2 + 1]) << 8) | (dataReceived[i + 10 + j * 2]);
                                        }
                                        break;
                                }

                                // Assign battery level
                                if (batt != null)
                                {
                                    tracker.BatteryLevel = (batt == 128) ? 1f : (batt.Value & 127);
                                }
                                // Battery voltage
                                if (batt_v != null)
                                {
                                    tracker.BatteryVoltage = (batt_v.Value + 245f) / 100f;
                                }
                                // Temperature
                                if (temp != null)
                                {
                                    tracker.Temperature = (temp > 0) ? (temp.Value / 2f - 39f) : (float?)null;
                                }

                                // Board Type
                                if (brd_id != null)
                                {
                                    var boardType = (BoardType)brd_id.Value;
                                    if (boardType != null)
                                    {
                                        device.BoardType = boardType;
                                    }
                                }

                                // MCU Type
                                if (mcu_id != null)
                                {
                                    var mcuType = (McuType)mcu_id.Value;
                                    if (mcuType != null)
                                    {
                                        device.McuType = mcuType;
                                    }
                                }

                                // Firmware version string
                                if (fw_date != null && fw_major != null && fw_minor != null && fw_patch != null)
                                {
                                    int firmwareYear = 2020 + ((fw_date.Value >> 9) & 127);
                                    int firmwareMonth = (fw_date.Value >> 5) & 15;
                                    int firmwareDay = fw_date.Value & 31;
                                    string firmwareDate = $"{firmwareYear:D4}-{firmwareMonth:D2}-{firmwareDay:D2}";
                                    device.FirmwareVersion = $"{fw_major}.{fw_minor}.{fw_patch} (Build {firmwareDate})";
                                }

                                // Tracker status
                                if (svr_status != null)
                                {
                                    var status = (TrackerStatus)svr_status.Value;
                                    if (status != null)
                                    {
                                        tracker.Status = status;
                                    }
                                }

                                // RSSI / Signal strength
                                if (rssi != null)
                                {
                                    tracker.SignalStrength = -rssi.Value;
                                }

                                // Rotation and acceleration
                                if (packetType == 1 || packetType == 4)
                                {
                                    // Convert Q15 short to float and reorder quaternion as x,y,z,w
                                    var rot = new Quaternion(
                                        q[0] / 32768f,
                                        q[1] / 32768f,
                                        q[2] / 32768f,
                                        q[3] / 32768f
                                    );

                                    Task.Run(() => tracker.SetRotation(rot));
                                }

                                if (packetType == 2)
                                {
                                    Task.Run(() =>
                                    {
                                        float[] v = new float[3];
                                        v[0] = q[0] / 1024f;
                                        v[1] = q[1] / 2048f;
                                        v[2] = q[2] / 2048f;

                                        for (int x = 0; x < 3; x++)
                                        {
                                            v[x] = v[x] * 2 - 1;
                                        }

                                        float d = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
                                        float invSqrtD = 1.0f / (float)Math.Sqrt(d + 1e-6f);
                                        float aAngle = (float)(Math.PI / 2) * d * invSqrtD;
                                        float s = (float)Math.Sin(aAngle);
                                        float k = s * invSqrtD;

                                        var rot = new Quaternion(
                                            k * v[0],
                                            k * v[1],
                                            k * v[2],
                                            (float)Math.Cos(aAngle)
                                        );
                                        tracker.SetRotation(rot);
                                    });
                                }

                                if (packetType == 1 || packetType == 2)
                                {
                                    float scaleAccel = 1f / (1 << 7);
                                    Vector3 acceleration = new Vector3(a[0], a[1], a[2]) * scaleAccel;
                                    Task.Run(() => tracker.SetAcceleration(Unsandwich(acceleration)));
                                }

                                if (packetType == 4)
                                {
                                    Vector3 magnetometer = new Vector3(m[0], m[1], m[2]) * (1000f / 1024f);
                                    device.MagnetometerStatus = MagnetometerStatus.ENABLED;
                                    Task.Run(() => tracker.SetMagVector(magnetometer));
                                }

                                if (packetType == 1 || packetType == 2 || packetType == 4)
                                {
                                    tracker.DataTick();
                                }
                            }
                        }

                        if (!devicesPresent)
                        {
                            Thread.Sleep(10);
                        }
                        else if (!devicesDataReceived)
                        {
                            Thread.Sleep(1);
                        }
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }
            }
        }
        public static Vector3 Unsandwich(Vector3 v)
        {
            Quaternion sensorOffsetCorrectionInv =
                Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f);

            Quaternion vQuat = new Quaternion(v, 0f);

            Quaternion result =
                sensorOffsetCorrectionInv * vQuat * Quaternion.Inverse(sensorOffsetCorrectionInv);

            return new Vector3(result.X, result.Y, result.Z);
        }
    }
}