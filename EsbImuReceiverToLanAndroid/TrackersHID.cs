// TrackersHID_Android.cs
using Android.App;
using Android.Content;
using Android.Hardware.Usb;
using Android.OS;
using System.Numerics;
using System.Text;
using EsbImuReceiverToLan.Tracking.Trackers.HID;
using Application = Android.App.Application;
using static Everything_To_IMU_SlimeVR.SlimeVR.FirmwareConstants;
using Java.Lang;
using Thread = System.Threading.Thread;
using Math = System.Math;
using Exception = System.Exception;
using System.Threading;
using Java.Nio;
namespace EsbImuReceiverToLan.Tracking.Trackers.HID {
    public class TrackersHID_Android {
        private const int HID_TRACKER_RECEIVER_VID = 0x1209;
        private const int HID_TRACKER_RECEIVER_PID = 0x7690;
        private const int PACKET_SIZE = 16;

        private UsbManager usbManager;
        private UsbDeviceConnection usbConnection;
        private UsbEndpoint endpointIn;

        private Thread dataReadThread;

        public event EventHandler<Tracker> trackersConsumer;
        private Quaternion AXES_OFFSET;

        private readonly Dictionary<int, Tracker> activeTrackers = new();
        bool deviceOpened = false;
        bool alreadyRunning = false;
        public TrackersHID_Android() {
            if (!alreadyRunning) {
                alreadyRunning = true;
                usbManager = (UsbManager)Application.Context.GetSystemService(Context.UsbService);
                SetupAxesOffset();
                Task.Run(() => {
                    DeviceEnumerateLoop();
                });
            }
        }

        private void SetupAxesOffset() {
            float angle = -MathF.PI / 2;
            AXES_OFFSET = Quaternion.CreateFromAxisAngle(Vector3.UnitX, angle);
        }
        private void DeviceEnumerateLoop() {
            Thread.Sleep(100); // Delayed start
            while (true) {
                Thread.Sleep(1000);
                EnumerateDevices();
            }
        }
        private void EnumerateDevices() {
            foreach (var entry in usbManager.DeviceList) {
                var device = entry.Value;
                if (device.VendorId == HID_TRACKER_RECEIVER_VID && device.ProductId == HID_TRACKER_RECEIVER_PID) {
                    if (!deviceOpened) {
                        if (!usbManager.HasPermission(device)) {
                            RequestPermission(device);
                            return;
                        }
                        OpenDevice(device);
                    }
                }
            }
        }

        private UsbPermissionReceiver usbPermissionReceiver;

        private void RequestPermission(UsbDevice device) {
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
            } else {
                Application.Context.RegisterReceiver(
                    new UsbPermissionReceiver(OpenDevice),
                    filter);
            }


            usbManager.RequestPermission(device, permissionIntent);
        }


        private void OpenDevice(UsbDevice device) {
            var connection = usbManager.OpenDevice(device);
            if (connection == null) {
                Console.WriteLine("[TrackersHID_Android] Failed to open USB device.");
                return;
            }
            usbConnection = connection;
            UsbInterface usbInterface = device.GetInterface(0);

            bool claimed = connection.ClaimInterface(usbInterface, true);
            if (!claimed) {
                Console.WriteLine("[TrackersHID_Android] Failed to claim interface.");
                return;
            }

            for (int i = 0; i < usbInterface.EndpointCount; i++) {
                var endpoint = usbInterface.GetEndpoint(i);
                if (endpoint.Direction == UsbAddressing.In) {
                    endpointIn = endpoint;
                }
            }

            if (endpointIn == null) {
                Console.WriteLine("[TrackersHID_Android] No IN interrupt endpoint found.");
                connection.ReleaseInterface(usbInterface);
                connection.Close();
                return;
            }

            deviceOpened = true;
            dataReadThread = new Thread(DataRead) { IsBackground = true };
            dataReadThread.Start();
        }

        private void DataRead() {

            ByteBuffer buffer = ByteBuffer.AllocateDirect(64);
            UsbRequest request = new UsbRequest();
            // HID Get Report request, often used to get first packet or trigger stream
            byte requestType = 0xA1; // Device-to-host | Class | Interface
            byte report = 0x01;     // GET_REPORT
            short value = 0x0100;    // Report Type (Input), Report ID (0)
            short index = 0;         // Interface number
            byte[] reportBuffer = new byte[64];
            int received = usbConnection.ControlTransfer(UsbAddressing.In, report, value, index, reportBuffer, reportBuffer.Length, 1000);
            while (true) {
                try {
                    buffer.Clear(); // Prepare buffer for new data
                    try {
                        request.Initialize(usbConnection, endpointIn);
                    } catch (Exception ex) {
                        Console.WriteLine($"[TrackersHID_Android] Failed to initialize UsbRequest: {ex.Message}");
                        return;
                    }

                    if (!request.Queue(buffer, 64)) {
                        Console.WriteLine("[TrackersHID_Android] Failed to queue UsbRequest");
                        Thread.Sleep(10);
                        continue;
                    }

                    UsbRequest completed = usbConnection.RequestWait();
                    if (completed == null) {
                        Console.WriteLine("[TrackersHID_Android] RequestWait returned null, device may have disconnected");
                        deviceOpened = false;
                        break;
                    }

                    if (completed.Equals(request)) {
                        buffer.Rewind(); // Set position to 0 to read from buffer start

                        int len = buffer.Limit();
                        byte[] data = new byte[len];
                        buffer.Get(data, 0, len);

                        ParsePacket(data, len);
                    } else {
                        Thread.Sleep(5);
                    }
                } catch (Exception ex) {
                    Console.WriteLine($"[TrackersHID_Android] Read error: {ex.Message}");
                    deviceOpened = false;
                    Thread.Sleep(10);
                    break;
                }
            }
        }

        private void ParsePacket(byte[] data, int length) {
            if (length < PACKET_SIZE || length % PACKET_SIZE != 0) {
                Console.WriteLine("[TrackersHID_Android] Malformed packet.");
                return;
            }

            for (int i = 0; i < length; i += PACKET_SIZE) {
                int packetType = data[i];
                int id = data[i + 1];
                int[] q = new int[4];
                int[] a = new int[3];
                int[] m = new int[3];

                int? batt = null, batt_v = null, temp = null, brd_id = null, mcu_id = null;
                int? fw_date = null, fw_major = null, fw_minor = null, fw_patch = null;
                int? svr_status = null, rssi = null;

                if (!activeTrackers.TryGetValue(id, out var tracker)) {
                    tracker = new Tracker(
                        new AndroidHidDevice(),
                        0,
                        $"/Tracker{id}",
                        $"Tracker{id}",
                        true,
                        true,
                        true,
                        ImuType.UNKNOWN,
                        true,
                        false,
                        false,
                        false,
                        MagnetometerStatus.DISABLED);

                    activeTrackers[id] = tracker;
                    trackersConsumer?.Invoke(this, tracker);
                }

                switch (packetType) {
                    case 0:
                        batt = data[i + 2];
                        batt_v = data[i + 3];
                        temp = data[i + 4];
                        brd_id = data[i + 5];
                        mcu_id = data[i + 6];
                        fw_date = (data[i + 11] << 8) | data[i + 10];
                        fw_major = data[i + 12];
                        fw_minor = data[i + 13];
                        fw_patch = data[i + 14];
                        rssi = data[i + 15];
                        break;

                    case 1:
                    case 4:
                        for (int j = 0; j < 4; j++)
                            q[j] = (short)((data[i + 2 + j * 2 + 1] << 8) | data[i + 2 + j * 2]);
                        var rot = new Quaternion(q[0] / 32768f, q[1] / 32768f, q[2] / 32768f, q[3] / 32768f);
                        tracker.SetRotation(rot);
                        break;

                    case 2:
                        batt = data[i + 2];
                        batt_v = data[i + 3];
                        temp = data[i + 4];
                        byte[] qBytes = new byte[4];
                        Array.Copy(data, i + 5, qBytes, 0, 4);
                        uint qBuf = BitConverter.ToUInt32(qBytes, 0);
                        float[] v = new float[3];
                        v[0] = (qBuf & 1023) / 1024f;
                        v[1] = ((qBuf >> 10) & 2047) / 2048f;
                        v[2] = ((qBuf >> 21) & 2047) / 2048f;

                        for (int x = 0; x < 3; x++)
                            v[x] = v[x] * 2 - 1;

                        float d = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
                        float invSqrtD = 1.0f / (float)Math.Sqrt(d + 1e-6f);
                        float aAngle = (float)(Math.PI / 2) * d * invSqrtD;
                        float s = (float)Math.Sin(aAngle);
                        float k = s * invSqrtD;

                        rot = new Quaternion(k * v[0], k * v[1], k * v[2], (float)Math.Cos(aAngle));
                        tracker.SetRotation(rot);
                        rssi = data[i + 15];
                        break;

                    case 3:
                        svr_status = data[i + 2];
                        rssi = data[i + 15];
                        break;
                }

                if (batt != null)
                    tracker.BatteryLevel = (batt == 128) ? 1f : (batt.Value & 127);
                if (batt_v != null)
                    tracker.BatteryVoltage = (batt_v.Value + 245f) / 100f;
                if (temp != null)
                    tracker.Temperature = (temp > 0) ? (temp.Value / 2f - 39f) : (float?)null;
                if (rssi != null)
                    tracker.SignalStrength = -rssi.Value;

                if (packetType == 1 || packetType == 2) {
                    for (int j = 0; j < 3; j++)
                        a[j] = (short)((data[i + 10 + j * 2 + 1] << 8) | data[i + 10 + j * 2]);
                    Vector3 accel = new Vector3(a[0], a[1], a[2]) * (1f / 128f);
                    tracker.SetAcceleration(accel);
                }

                if (packetType == 4) {
                    for (int j = 0; j < 3; j++)
                        m[j] = (short)((data[i + 10 + j * 2 + 1] << 8) | data[i + 10 + j * 2]);
                    Vector3 mag = new Vector3(m[0], m[1], m[2]) * (1000f / 1024f);
                    tracker.SetMagVector(mag);
                }
            }
        }

        private class UsbPermissionReceiver : BroadcastReceiver {
            private readonly Action<UsbDevice> onGranted;

            public UsbPermissionReceiver(Action<UsbDevice> onGranted) {
                this.onGranted = onGranted;
            }

            public override void OnReceive(Context context, Intent intent) {
                if (intent.Action == "com.SebaneStudios.USB_PERMISSION") {
                    UsbDevice device = (UsbDevice)intent.GetParcelableExtra(UsbManager.ExtraDevice);

                    bool granted = intent.GetBooleanExtra(UsbManager.ExtraPermissionGranted, false);
                    Console.WriteLine($"[UsbPermissionReceiver] Permission result for device {device?.DeviceName}: granted = {granted}");

                    if (granted && device != null) {
                        onGranted?.Invoke(device);
                    } else {
                        Console.WriteLine("[UsbPermissionReceiver] USB permission denied.");
                    }

                    // IMPORTANT: Unregister the receiver once permission response received to avoid leaks
                    try {
                        context.UnregisterReceiver(this);
                        Console.WriteLine("[UsbPermissionReceiver] Receiver unregistered.");
                    } catch (IllegalArgumentException e) {
                        Console.WriteLine("[UsbPermissionReceiver] Receiver already unregistered or not registered.");
                    }
                } else {
                    Console.WriteLine($"[UsbPermissionReceiver] Received unexpected intent: {intent.Action}");
                }
            }
        }
    }
}