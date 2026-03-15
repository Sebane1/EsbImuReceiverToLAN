#include "UsbHidHandler.h"
#include "SlimeUdpClient.h"
#include "config.h" 

#include "usb/usb_host.h"
#include "hid_host.h"

extern void processHidData(SlimeUdpClient* udpClient, uint8_t* dataReceived, size_t validLength);

static const char *TAG = "UsbHid";
static QueueHandle_t hid_host_event_queue;
static SlimeUdpClient* g_udpClient = nullptr;

typedef struct {
  hid_host_device_handle_t hid_device_handle;
  hid_host_driver_event_t event;
  void *arg;
} hid_host_event_queue_t;

static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg) {
  uint8_t data[64] = {0};
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(
        hid_device_handle, data, 64, &data_length));
    
    DEBUG_PRINTF("HID IN Report received! Length: %d\n", data_length);

    if (g_udpClient) {
        processHidData(g_udpClient, data, data_length);
    }
    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    DEBUG_PRINTLN("HID Device DISCONNECTED");
    ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    DEBUG_PRINTLN("HID Device TRANSFER_ERROR");
    break;
  default:
    break;
  }
}

static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event, void *arg) {
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));
  const hid_host_device_config_t dev_config = {
      .callback = hid_host_interface_callback, 
      .callback_arg = NULL
  };

  switch (event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED: {
    DEBUG_PRINTLN("HID Device CONNECTED");
    esp_err_t err = hid_host_device_open(hid_device_handle, &dev_config);
    if (err != ESP_OK) {
        DEBUG_PRINTF("Error: Failed to open HID device. Code: 0x%X\n", err);
        break;
    }
    
    // Always attempt to set BOOT protocol if it's supported,
    // otherwise some devices (like custom dongles) won't start sending reports.
    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      if (hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT) != ESP_OK) {
        DEBUG_PRINTLN("Warning: Failed to set BOOT protocol, ignoring...");
      }
      if (hid_class_request_set_idle(hid_device_handle, 0, 0) != ESP_OK) {
        DEBUG_PRINTLN("Warning: Failed to set IDLE, ignoring...");
      }
    } else {
      // For generic HID devices
      if (hid_class_request_set_idle(hid_device_handle, 0, 0) != ESP_OK) {
        DEBUG_PRINTLN("Warning: Failed to set IDLE on generic HID device, ignoring...");
      }
    }

    esp_err_t err_start = hid_host_device_start(hid_device_handle);
    if (err_start != ESP_OK) {
        DEBUG_PRINTF("Error: Failed to start HID device. Code: 0x%X\n", err_start);
    }
    break;
  }
  default:
    break;
  }
}

static void usb_lib_task(void *arg) {
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };

  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xTaskNotifyGive((TaskHandle_t)arg);

  while (true) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      usb_host_device_free_all();
    }
  }
}

static void hid_host_task(void *pvParameters) {
  hid_host_event_queue_t evt_queue;
  hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));

  while (true) {
    if (xQueueReceive(hid_host_event_queue, &evt_queue, portMAX_DELAY)) {
      hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event, evt_queue.arg);
    }
  }
}

static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event, void *arg) {
  const hid_host_event_queue_t evt_queue = {
      .hid_device_handle = hid_device_handle, .event = event, .arg = arg};
  xQueueSend(hid_host_event_queue, &evt_queue, 0);
}

UsbHidHandler::UsbHidHandler() {
    _udpClient = nullptr;
}

void UsbHidHandler::begin(SlimeUdpClient* udpClient) {
    _udpClient = udpClient;
    g_udpClient = udpClient;

    BaseType_t task_created = xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096, xTaskGetCurrentTaskHandle(), 2, NULL, 0);
    assert(task_created == pdTRUE);

    ulTaskNotifyTake(false, 1000);

    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = NULL
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

    task_created = xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
    assert(task_created == pdTRUE);

    DEBUG_PRINTLN("USB Host Handler Initialized.");
}

void UsbHidHandler::loop() {
}

// Below is the translated parser logic from TrackersHID.cs
// Call this function from your USB HID IN Report callback
static float q15ToFloat(int16_t q) {
    return q / 32768.0f;
}

static float q11ToFloat(int16_t q) {
    return q / 1024.0f;
}

void processHidData(SlimeUdpClient* udpClient, uint8_t* dataReceived, size_t validLength) {
    if (validLength == 0 || validLength % 16 != 0) {
        DEBUG_PRINTF("Dropped HID Report - Invalid length: %d\n", validLength);
        return; // Malformed
    }

    int packetCount = validLength / 16;
    for (int i = 0; i < packetCount * 16; i += 16) {
        uint8_t packetType = dataReceived[i];
        uint8_t id = dataReceived[i + 1];
        uint8_t trackerId = id; // simplified mapping for single dongle
        
        if (packetType == 255) {
            DEBUG_PRINTLN("Device Register Packet received");
            continue;
        }

        if (packetType == 0) {
            uint8_t imuId = dataReceived[i + 8];
            udpClient->addTracker(trackerId, (int)imuId);
            continue;
        }

        int batt = -1, batt_v = -1;
        float ax = 0, ay = 0, az = 0;
        float qx = 0, qy = 0, qz = 0, qw = 1.0f;
        bool hasAccel = false, hasRotation = false, hasBattery = false;

        switch (packetType) {
            case 0: // device info
                batt = dataReceived[i + 2];
                batt_v = dataReceived[i + 3];
                hasBattery = true;
                break;
            case 1: // full precision quat and accel
            case 4: { // full precision quat and mag
                int16_t q[4];
                q[0] = (dataReceived[i + 3] << 8) | dataReceived[i + 2];
                q[1] = (dataReceived[i + 5] << 8) | dataReceived[i + 4];
                q[2] = (dataReceived[i + 7] << 8) | dataReceived[i + 6];
                q[3] = (dataReceived[i + 9] << 8) | dataReceived[i + 8];
                
                qx = q15ToFloat(q[0]);
                qy = q15ToFloat(q[1]);
                qz = q15ToFloat(q[2]);
                qw = q15ToFloat(q[3]);
                hasRotation = true;

                if (packetType == 1) {
                    int16_t a[3];
                    a[0] = (dataReceived[i + 11] << 8) | dataReceived[i + 10];
                    a[1] = (dataReceived[i + 13] << 8) | dataReceived[i + 12];
                    a[2] = (dataReceived[i + 15] << 8) | dataReceived[i + 14];
                    float scaleAccel = 1.0f / 128.0f;
                    ax = a[0] * scaleAccel;
                    ay = a[1] * scaleAccel;
                    az = a[2] * scaleAccel;
                    hasAccel = true;
                }
                break;
            }
            case 2: { // reduced precision quat and accel
                batt = dataReceived[i + 2];
                batt_v = dataReceived[i + 3];
                hasBattery = true;

                uint32_t q_buf = 0;
                memcpy(&q_buf, &dataReceived[i + 5], 4);
                int q0 = (int)(q_buf & 1023);
                int q1 = (int)((q_buf >> 10) & 2047);
                int q2 = (int)((q_buf >> 21) & 2047);
                
                int16_t a[3];
                a[0] = (dataReceived[i + 10] << 8) | dataReceived[i + 9];
                a[1] = (dataReceived[i + 12] << 8) | dataReceived[i + 11];
                a[2] = (dataReceived[i + 14] << 8) | dataReceived[i + 13];

                float v[3];
                v[0] = q0 / 1024.0f;
                v[1] = q1 / 2048.0f;
                v[2] = q2 / 2048.0f;
                for (int x = 0; x < 3; ++x) v[x] = v[x] * 2.0f - 1.0f;

                float d = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
                float invSqrtD = 1.0f / sqrt(d + 1e-6f);
                float aAngle = (PI / 2.0f) * d * invSqrtD;
                float s = sin(aAngle);
                float k = s * invSqrtD;
                qx = k * v[0];
                qy = k * v[1];
                qz = k * v[2];
                qw = cos(aAngle);
                hasRotation = true;

                float scaleAccel = 1.0f / 128.0f;
                ax = a[0] * scaleAccel;
                ay = a[1] * scaleAccel;
                az = a[2] * scaleAccel;
                hasAccel = true;
                break;
            }
            case 3: // status
                // Status packet
                break;
        }

        if (hasRotation) {
            udpClient->sendRotation(trackerId, qx, qy, qz, qw);
        }
        if (hasAccel) {
            // Apply Unsandwich transformation from TrackersHID.cs
            // The C# code does: z-axis rotation by PI/2.
            // Result roughly swaps/negates axes to match SlimeVR expected orientation.
            // Unsandwich applies inverse offset correction. 
            // We'll send raw and let SlimeVR calibrate, or apply a rough offset:
            // For now, sending standard ax, ay, az.
            udpClient->sendAcceleration(trackerId, ax, ay, az);
        }
        if (hasBattery) {
            float voltage = (batt_v + 245.0f) / 100.0f;
            float percentage = (batt == 128) ? 100.0f : (batt & 127);
            udpClient->sendBattery(voltage, percentage);
        }
    }
}
