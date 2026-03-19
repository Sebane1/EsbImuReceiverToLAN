#include "UsbHidHandler.h"
#include "SlimeUdpClient.h"
#include "config.h"

#include "hid_host.h"
#include "usb/usb_host.h"
#include <map>
#include <utility>

extern void processHidData(SlimeUdpClient *udpClient,
                           hid_host_device_handle_t hid_device,
                           uint8_t *dataReceived, size_t validLength);

static const char *TAG = "UsbHid";
static QueueHandle_t hid_host_event_queue;
static SlimeUdpClient *g_udpClient = nullptr;

typedef struct {
  hid_host_device_handle_t hid_device_handle;
  hid_host_driver_event_t event;
  void *arg;
} hid_host_event_queue_t;

typedef struct {
  hid_host_device_handle_t hid_device;
  uint8_t data[64];
  size_t length;
} hid_report_t;

#define HID_REPORT_POOL_SIZE 32
static hid_report_t hid_report_pool[HID_REPORT_POOL_SIZE];
static QueueHandle_t hid_data_queue = NULL;
static QueueHandle_t free_report_queue = NULL;

static void
hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                            const hid_host_interface_event_t event, void *arg) {
  uint8_t data[64] = {0};
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT: {
    if (!hid_data_queue || !free_report_queue)
      break;

    hid_report_t *report = NULL;
    // Pop a free buffer pointer from the pool queue
    if (xQueueReceive(free_report_queue, &report, 0) == pdTRUE) {
      size_t data_length = 0;
      esp_err_t err = hid_host_device_get_raw_input_report_data(
          hid_device_handle, report->data, 64, &data_length);

      if (err == ESP_OK && data_length > 0) {
        report->hid_device = hid_device_handle;
        report->length = (data_length > 64) ? 64 : data_length;

        // Send the pointer to the data queue
        if (xQueueSend(hid_data_queue, &report, 0) != pdTRUE) {
          // Data queue full, return to free pool
          xQueueSend(free_report_queue, &report, 0);
        }
      } else {
        // Error or no data, return buffer to free pool
        xQueueSend(free_report_queue, &report, 0);
      }
    }
    break;
  }

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
                                  const hid_host_driver_event_t event,
                                  void *arg) {
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));
  const hid_host_device_config_t dev_config = {
      .callback = hid_host_interface_callback, .callback_arg = NULL};

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
      if (hid_class_request_set_protocol(hid_device_handle,
                                         HID_REPORT_PROTOCOL_BOOT) != ESP_OK) {
        DEBUG_PRINTLN("Warning: Failed to set BOOT protocol, ignoring...");
      }
      if (hid_class_request_set_idle(hid_device_handle, 0, 0) != ESP_OK) {
        DEBUG_PRINTLN("Warning: Failed to set IDLE, ignoring...");
      }
    } else {
      // For generic HID devices
      if (hid_class_request_set_idle(hid_device_handle, 0, 0) != ESP_OK) {
        DEBUG_PRINTLN(
            "Warning: Failed to set IDLE on generic HID device, ignoring...");
      }
    }

    esp_err_t err_start = hid_host_device_start(hid_device_handle);
    if (err_start != ESP_OK) {
      DEBUG_PRINTF("Error: Failed to start HID device. Code: 0x%X\n",
                   err_start);
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
      hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event,
                            evt_queue.arg);
    }
  }
}

static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_driver_event_t event,
                                     void *arg) {
  const hid_host_event_queue_t evt_queue = {
      .hid_device_handle = hid_device_handle, .event = event, .arg = arg};
  xQueueSend(hid_host_event_queue, &evt_queue, 0);
}

UsbHidHandler::UsbHidHandler() {
  _udpClient = nullptr;
  _lastReportTime = 0;
}

void UsbHidHandler::begin(SlimeUdpClient *udpClient) {
  _udpClient = udpClient;
  g_udpClient = udpClient;
  _lastReportTime = millis();

  // Create queues to hold POINTERS to buffers (minimizes copying)
  hid_data_queue = xQueueCreate(HID_REPORT_POOL_SIZE, sizeof(hid_report_t *));
  free_report_queue =
      xQueueCreate(HID_REPORT_POOL_SIZE, sizeof(hid_report_t *));

  // Pre-seed the free queue with pointers to ALL buffers in our pre-allocated
  // pool
  for (int i = 0; i < HID_REPORT_POOL_SIZE; i++) {
    hid_report_t *report = &hid_report_pool[i];
    xQueueSend(free_report_queue, &report, 0);
  }

  BaseType_t task_created =
      xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                              xTaskGetCurrentTaskHandle(), 2, NULL, 0);
  assert(task_created == pdTRUE);

  ulTaskNotifyTake(false, 1000);

  const hid_host_driver_config_t hid_host_driver_config = {
      .create_background_task = true,
      .task_priority = 5,
      .stack_size = 4096,
      .core_id = 0,
      .callback = hid_host_device_callback,
      .callback_arg = NULL};
  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  task_created =
      xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
  assert(task_created == pdTRUE);

  DEBUG_PRINTLN("USB Host Handler Initialized.");
}

void UsbHidHandler::loop() {
  if (!hid_data_queue || !free_report_queue || !_udpClient)
    return;

  hid_report_t *report = NULL;
  // Process all pending report pointers off the data queue
  while (xQueueReceive(hid_data_queue, &report, 0) == pdTRUE) {
    processHidData(_udpClient, report->hid_device, report->data,
                   report->length);
    updateActivity();

    /* Recycling: Push the pointer back to the free queue for reuse by the USB
     * stack*/
    xQueueSend(free_report_queue, &report, 0);
  }
}

// Translated parser logic from TrackersHID.cs
static float q15ToFloat(int16_t q) { return q / 32768.0f; }

static float q11ToFloat(int16_t q) { return q / 1024.0f; }

void processHidData(SlimeUdpClient *udpClient,
                    hid_host_device_handle_t hid_device, uint8_t *dataReceived,
                    size_t validLength) {
  if (validLength == 0 || validLength % 16 != 0) {
    DEBUG_PRINTF("Dropped HID Report - Invalid length: %d\n", validLength);
    return; // Malformed
  }

  // A map to keep track of which hardware ID maps to which trackerIndex (0-39)
  // Handle-Agnostic: We use only the 1-byte ESB 'id' as the key.
  static std::map<uint8_t, uint8_t> dongleIdToTrackerIndex;
  static uint8_t nextTrackerIndex = 0;
  static uint8_t storedMacs[40][6] = {0};

  int packetCount = validLength / 16;
  for (int i = 0; i < packetCount * 16; i += 16) {
    uint8_t packetType = dataReceived[i];
    uint8_t id = dataReceived[i + 1];

    if (packetType == 255) {
      if (dongleIdToTrackerIndex.find(id) == dongleIdToTrackerIndex.end() &&
          nextTrackerIndex < 15) {
        DEBUG_PRINTF(
            "New Device Registered: ID %d assigning Tracker Index %d\n", id,
            nextTrackerIndex);
        dongleIdToTrackerIndex[id] = nextTrackerIndex++;
      }

      if (dongleIdToTrackerIndex.find(id) != dongleIdToTrackerIndex.end()) {
        uint8_t tIdx = dongleIdToTrackerIndex[id];
        // MAC Address is in bytes 2-7
        memcpy(storedMacs[tIdx], &dataReceived[i + 2], 6);
      }
      continue;
    }

    if (dongleIdToTrackerIndex.find(id) == dongleIdToTrackerIndex.end()) {
      // We haven't seen a register packet for this tracker ID yet, ignore data
      continue;
    }

    uint8_t trackerIndex = dongleIdToTrackerIndex[id];

    if (packetType == 0) {
      uint8_t imuId = dataReceived[i + 8];

      int brd_id = dataReceived[i + 5];
      int mcu_id = dataReceived[i + 6];

      int fw_major = dataReceived[i + 12];
      int fw_minor = dataReceived[i + 13];
      int fw_patch = dataReceived[i + 14];

      char firmwareStr[32];
      snprintf(firmwareStr, sizeof(firmwareStr), "ESB %d.%d.%d", fw_major,
               fw_minor, fw_patch);

      udpClient->initializeTracker(trackerIndex, storedMacs[trackerIndex],
                                   (int)imuId, brd_id, mcu_id, firmwareStr);
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
    case 1:   // full precision quat and accel
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

      uint32_t q_buf = ((uint32_t)dataReceived[i + 5]) |
                       ((uint32_t)dataReceived[i + 6] << 8) |
                       ((uint32_t)dataReceived[i + 7] << 16) |
                       ((uint32_t)dataReceived[i + 8] << 24);

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
      for (int x = 0; x < 3; ++x)
        v[x] = v[x] * 2.0f - 1.0f;

      float d = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
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

    VirtualTracker *vt = udpClient->getTracker(trackerIndex);
    if (vt) {
      vt->lastSendDataTime = millis();

      // PPS Reduction: Alternate between Rotation and Acceleration
      // instead of sending both every frame (halves packet pressure)
      if (vt->updateCounter % 2 == 0) {
        if (hasRotation) {
          udpClient->sendRotation(trackerIndex, qx, qy, qz, qw);
        }
      } else {
        if (hasAccel) {
          udpClient->sendAcceleration(trackerIndex, ax, ay, az);
        }
      }
      vt->updateCounter++;

      if (hasBattery && batt != -1) {
        // Battery reporting is less time sensitive.
        // Only send every 10 seconds to avoid network flood.
        if (millis() - vt->lastBatterySendTime > 10000) {
          vt->lastBatterySendTime = millis();

          float percentage = (batt == 128) ? 100.0f : (batt & 127);
          if (percentage > 100.0f)
            percentage = 100.0f;

          float voltage = (batt_v >= 0) ? ((batt_v + 245.0f) / 100.0f) : 3.3f;
          udpClient->sendBattery(trackerIndex, voltage, percentage);
        }
      }

      // Staggering: Give the Wi-Fi driver time to handle each packet
      // to prevent filling the internal hardware queue too fast.
      delayMicroseconds(400);
    }
  }
}
