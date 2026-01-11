// TrackersHID_Android.cs
using Android.App;
using Android.Content;
using Android.Hardware.Usb;
using Android.OS;
using System.Numerics;
using System.Text;
using EsbImuReceiverToLan.Tracking.Trackers.HID;
using Application = Android.App.Application;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;
using Java.Lang;
using Thread = System.Threading.Thread;
using Math = System.Math;
using Exception = System.Exception;
using System.Threading;
using Java.Nio;
using static EsbImuReceiverToLan.Tracking.Trackers.HID.TrackersHID_Android;
using SlimeImuProtocol;
using SlimeImuProtocol.SlimeVR;
namespace EsbImuReceiverToLan.Tracking.Trackers.HID
{
    public class TrackersHID_Android : IDisposable
    {
        private const int HID_TRACKER_RECEIVER_VID = 0x1209;
        private const int HID_TRACKER_RECEIVER_PID = 0x7690;
        private const int PACKET_SIZE = 16;
        private readonly List<TrackerDevice> devices = new();
        private readonly Dictionary<string, List<int>> devicesBySerial = new();
        private readonly Dictionary<UsbEndpoint, List<int>> devicesByHID = new();
        private readonly Dictionary<UsbEndpoint, int> lastDataByHID = new();

        private UsbManager usbManager;
        private UsbDeviceConnection usbConnection;
        private UsbEndpoint endpointIn;

        private static Thread dataReadThread;

        public event EventHandler<Tracker> trackersConsumer;
        private Quaternion AXES_OFFSET;

        private readonly Dictionary<int, Tracker> activeTrackers = new();
        private UsbInterface usbInterfaceIn;
        bool deviceOpened = false;
        bool alreadyRunning = false;
        bool disposed = false;

        private UsbPermissionReceiver usbPermissionReceiver;
        private FunctionSequenceManager _functionSequenceManager;
        public TrackersHID_Android()
        {
            if (!alreadyRunning)
            {
                alreadyRunning = true;
                usbManager = (UsbManager)Application.Context.GetSystemService(Context.UsbService);
                SetupAxesOffset();
                _functionSequenceManager = new FunctionSequenceManager();
                Task.Run(() =>
                {
                    DeviceEnumerateLoop();
                });
            }
        }

        private void SetupAxesOffset()
        {
            float angle = -MathF.PI / 2;
            AXES_OFFSET = Quaternion.CreateFromAxisAngle(Vector3.UnitX, angle);
        }
        private void DeviceEnumerateLoop()
        {
            Thread.Sleep(100); // Delayed start
            while (!deviceOpened)
            {
                Thread.Sleep(1000);
                EnumerateDevices();
            }
        }
        private void EnumerateDevices()
        {
            foreach (var entry in usbManager.DeviceList)
            {
                var device = entry.Value;
                if (device.VendorId == HID_TRACKER_RECEIVER_VID && device.ProductId == HID_TRACKER_RECEIVER_PID)
                {
                    if (!deviceOpened)
                    {
                        if (!usbManager.HasPermission(device))
                        {
                            RequestPermission(device);
                            return;
                        }
                        OpenDevice(device);
                    }
                }
            }
        }

        private void RequestPermission(UsbDevice device)
        {
            var receiverNotExported = (PendingIntentFlags)0x08;

            var permissionIntent = PendingIntent.GetBroadcast(
                Application.Context,
                0,
                new Intent("com.SebaneStudios.USB_PERMISSION"),
                PendingIntentFlags.Immutable | receiverNotExported);

            var filter = new IntentFilter("com.SebaneStudios.USB_PERMISSION");

            if ((int)Build.VERSION.SdkInt >= 33) // Android 13+, technically needed from 31
            {
                // 0x2 = RECEIVER_NOT_EXPORTED
                Application.Context.RegisterReceiver(
                    new UsbPermissionReceiver(OpenDevice),
                    filter,
                    null,
                    null,
                    ReceiverFlags.NotExported); // RECEIVER_NOT_EXPORTED
            }
            else
            {
                Application.Context.RegisterReceiver(
                    new UsbPermissionReceiver(OpenDevice),
                    filter);
            }


            usbManager.RequestPermission(device, permissionIntent);
        }


        private void OpenDevice(UsbDevice device)
        {
            if (!deviceOpened)
            {
                var connection = usbManager.OpenDevice(device);
                if (connection == null)
                {
                    Console.WriteLine("[TrackersHID_Android] Failed to open USB device.");
                    return;
                }
                usbConnection = connection;
                UsbInterface usbInterface = device.GetInterface(2);
                UsbEndpoint ep = usbInterface.GetEndpoint(0);
                //if (ep.Type == UsbAddressing.XferBulk && ep.Direction == UsbAddressing.In) {
                endpointIn = ep;

                // Claim the interface AFTER we’ve found a valid endpoint
                bool claimed = connection.ClaimInterface(usbInterface, true);
                if (!claimed)
                {
                    Console.WriteLine("[TrackersHID_Android] Failed to claim interface.");
                    return;
                }

                if (!devicesByHID.ContainsKey(endpointIn))
                {
                    devicesByHID[endpointIn] = new List<int>();
                }
                // Start the read thread once the endpoint is set and claimed
                usbInterfaceIn = usbInterface; // Save this if you want to release later
                deviceOpened = true;
                dataReadThread = new Thread(DataRead) { IsBackground = true };
                dataReadThread.Start();
                return;
                //}
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
        private TrackerDevice DeviceIdLookup(UsbEndpoint hidDevice, int deviceId, string deviceName, List<int> deviceList)
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

                Console.WriteLine($"[TrackerServer] Added device {deviceName} for {hidDevice.Address}, id {deviceId}");

                return device;
            }
        }
        private void DataRead()
        {
            while (deviceOpened && !disposed)
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
                            if (!disposed)
                            {
                                var hidDevice = kvp.Key;
                                List<int> deviceList = kvp.Value;

                                byte[] dataReceived = new byte[64]; // 1 byte report ID + 64 bytes payload
                                try
                                {
                                    int result = usbConnection.BulkTransfer(endpointIn, dataReceived, dataReceived.Length, 500);
                                }
                                catch (Exception ex)
                                {
                                    Console.WriteLine($"[TrackersHID_Android] Read error: {ex.Message}");
                                    deviceOpened = false;
                                    Thread.Sleep(10);
                                    break;
                                }
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
                                        // Run this on a seperate thread in case its blocking?
                                        var localQuaternion = q;
                                        Task.Run(() =>
                                        {
                                            var threadCopyOfQuaternion = localQuaternion;
                                            float[] v = new float[3];
                                            v[0] = threadCopyOfQuaternion[0] / 1024f;
                                            v[1] = threadCopyOfQuaternion[1] / 2048f;
                                            v[2] = threadCopyOfQuaternion[2] / 2048f;

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

                            /*if (!devicesPresent)
                            {
                                Thread.Sleep(10);
                            }
                            else if (!devicesDataReceived)
                            {
                                Thread.Sleep(1);
                            }*/
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


        internal void StopReading()
        {
            disposed = true;
            deviceOpened = false;
            try
            {
                dataReadThread?.Interrupt();
                dataReadThread = null;
                usbConnection?.ReleaseInterface(usbInterfaceIn);
                usbConnection?.Close();
                usbConnection = null;
                usbInterfaceIn = null;
                endpointIn = null;
                devicesByHID?.Clear();
                foreach (var device in devices)
                {
                    foreach (var tracker in device.Trackers)
                    {
                        tracker.Value?.Dispose();
                    }
                    device.Trackers?.Clear();
                }
                devices.Clear();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Service] Cleanup error: {ex.Message}");
            }
        }

        public void Dispose()
        {
            StopReading();
        }

        private class UsbPermissionReceiver : BroadcastReceiver
        {
            private readonly Action<UsbDevice> onGranted;

            public UsbPermissionReceiver(Action<UsbDevice> onGranted)
            {
                this.onGranted = onGranted;
            }

            public override void OnReceive(Context context, Intent intent)
            {
                if (intent.Action == "com.SebaneStudios.USB_PERMISSION")
                {
                    UsbDevice device = (UsbDevice)intent.GetParcelableExtra(UsbManager.ExtraDevice);

                    bool granted = intent.GetBooleanExtra(UsbManager.ExtraPermissionGranted, false);
                    Console.WriteLine($"[UsbPermissionReceiver] Permission result for device {device?.DeviceName}: granted = {granted}");

                    if (granted && device != null)
                    {
                        onGranted?.Invoke(device);
                    }
                    else
                    {
                        Console.WriteLine("[UsbPermissionReceiver] USB permission denied.");
                    }

                    // IMPORTANT: Unregister the receiver once permission response received to avoid leaks
                    try
                    {
                        context.UnregisterReceiver(this);
                        Console.WriteLine("[UsbPermissionReceiver] Receiver unregistered.");
                    }
                    catch (IllegalArgumentException e)
                    {
                        Console.WriteLine("[UsbPermissionReceiver] Receiver already unregistered or not registered.");
                    }
                }
                else
                {
                    Console.WriteLine($"[UsbPermissionReceiver] Received unexpected intent: {intent.Action}");
                }
            }
        }
    }
}