#include "SlimeUdpClient.h"
#include "SerialManager.h"
#include "config.h"
#include <WiFi.h>
#include <string.h>
#include <vector>


// Safe float-to-uint32 bitcast using union (avoids Xtensa alignment crash from
// memcpy on register-resident floats)
static inline uint32_t float_to_uint32(float f) {
  union {
    float f;
    uint32_t u;
  } conv;
  conv.f = f;
  return conv.u;
}

SlimeUdpClient::SlimeUdpClient() {
  _protocolVersion = 22;
  _serverPort = 6969;
}

void SlimeUdpClient::begin(const char *serverIp, uint16_t serverPort) {
  if (serverIp && strlen(serverIp) > 0 && String(serverIp) != "0.0.0.0") {
    _serverIp.fromString(serverIp);
    _discoveryMode = false;
  } else {
    _discoveryMode = true;
    _serverIp = IPAddress(0, 0, 0, 0);
  }
  _serverPort = serverPort;
  randomSeed(micros());
  DEBUG_PRINTLN("SlimeUdpClient Initialized");
}

static bool _globalHandshakeSuccess = false;

bool SlimeUdpClient::isHandshakeSuccessful() { return _globalHandshakeSuccess; }

void SlimeUdpClient::onWiFiDisconnect() {
  _discoveryMode = true;
  _globalHandshakeSuccess = false;
  DEBUG_PRINTLN("[SlimeVR] WiFi Disconnected, resetting handshake state.");
  for (int i = 0; i < 16; i++) {
    _trackers[i].isInitialized = false;
    _trackers[i].handshakeOngoing = false;
    _trackers[i].udp.stop(); // Always stop to be safe
  }
}

void SlimeUdpClient::onWiFiConnect() {
  DEBUG_PRINTLN("[SlimeVR] WiFi Connected, preparing ritual trackers...");
  _globalHandshakeSuccess = false;
  _discoveryMode = true;
  _serverIp =
      IPAddress(0, 0, 0, 0); // Clear cached server IP for fresh discovery

  uint8_t mac[6];
  WiFi.macAddress(mac);

  for (int i = 0; i < 16; i++) {
    // reserve index 15 for the Hub tracker (the Stealth Probe)
    if (i == 15) {
      _trackers[i].active = true;
      memcpy(_trackers[i].hardwareAddress, mac, 6);
      _trackers[i].imuType = 0;
      _trackers[i].boardType = 0;
      _trackers[i].mcuType = 4;
      strncpy(_trackers[i].firmware, "ESP32-Hub", 31);
    }

    if (_trackers[i].active) {
      _trackers[i].lastPacketReceivedTime = millis();
      _trackers[i].lastHeartbeatTime = millis();
      _trackers[i].lastSendDataTime = 0;
      _trackers[i].lastBatterySendTime = 0;
      _trackers[i].handshakeRetryCount = 0;
      _trackers[i].isInitialized = false;
      _trackers[i].handshakeOngoing = true;
      _trackers[i].lastHandshakeTime = 0;
      _trackers[i].packetId = 0; // Reset packet sequence for new session

      uint16_t randomPort = 50000 + random(0, 10000);
      _trackers[i].udp.stop();
      if (!_trackers[i].udp.begin(randomPort)) {
        DEBUG_PRINTF("FAILED to bind UDP port %d for Tracker %d\n", randomPort,
                     i);
      }
    }
  }
}

void SlimeUdpClient::initializeTracker(uint8_t trackerIndex,
                                       const uint8_t mac[6], int imuType,
                                       int boardType, int mcuType,
                                       const char *firmwareVersion) {
  if (trackerIndex >= 16)
    return;
  VirtualTracker &vt = _trackers[trackerIndex];
  if (vt.active)
    return;

  vt.active = true;
  memcpy(vt.hardwareAddress, mac, 6);
  vt.packetId = 0;
  vt.handshakeOngoing = true;
  vt.isInitialized = false;
  vt.imuType = imuType;
  vt.boardType = boardType;
  vt.mcuType = mcuType;
  strncpy(vt.firmware, firmwareVersion, sizeof(vt.firmware) - 1);
  vt.firmware[sizeof(vt.firmware) - 1] = '\0'; // Ensure null-termination
  vt.lastHeartbeatTime = millis();
  vt.lastHandshakeTime = millis();

  // Bind the socket
  if (trackerIndex < 16) {
    uint16_t randomPort = 50000 + random(0, 10000);
    if (!vt.udp.begin(randomPort)) {
      DEBUG_PRINTF("FAILED to bind UDP port %d for Tracker %d (initialize)\n",
                   randomPort, trackerIndex);
    }
  }

  sendHandshake(trackerIndex, vt.firmware);
}

void SlimeUdpClient::loop() {
  for (int i = 0; i < 16; i++) {
    VirtualTracker &vt = _trackers[i];
    if (!vt.active)
      continue;

    if (vt.handshakeOngoing) {
      uint32_t now = millis();

      // Handshake Cooldown: If a timeout just happened (lastTimeoutTime > 0),
      // strictly IGNORE all handshake attempts for the configured period (30s).
      if (vt.lastTimeoutTime > 0 &&
          (now - vt.lastTimeoutTime < HANDSHAKE_COOLDOWN_MS)) {
        continue;
      }

      // Staggered handshake: All trackers handshake in parallel, but with
      // slightly different base intervals to avoid overwhelming the server
      // stack with simultaneous discovery bursts.
      if (now - vt.lastHandshakeTime > (1000 + (i * 20))) {
        vt.lastHandshakeTime = now;
        vt.handshakeRetryCount++;

        if (vt.handshakeRetryCount % 10 == 0) {
          uint16_t newPort = 50000 + random(0, 10000);
          DEBUG_PRINTF("Resetting socket for Tracker %d (New Port: %d) after "
                       "%d retries\n",
                       i, newPort, vt.handshakeRetryCount);
          vt.udp.stop();
          if (!vt.udp.begin(newPort)) {
            DEBUG_PRINTF("FAILED to re-bind UDP port %d for Tracker %d\n",
                         newPort, i);
          }
        }

        DEBUG_PRINTF("Sending Handshake for Tracker %d (Try %d)\n", i,
                     vt.handshakeRetryCount);
        sendHandshake(i, vt.firmware);
      }
    }

    if (vt.isInitialized && (millis() - vt.lastPacketReceivedTime > 5000)) {
      DEBUG_PRINTF(
          "Connection TIMEOUT for Tracker %d - Cooldown active (%ds)\n", i,
          HANDSHAKE_COOLDOWN_MS / 1000);
      vt.isInitialized = false;
      vt.handshakeOngoing = true;
      vt.handshakeRetryCount = 0;
      vt.lastHandshakeTime = 0;
      vt.lastTimeoutTime = millis(); // Start the cooldown
    }

    // Heartbeat: Send every 900ms to keep the connection alive.
    // (Index 15) skips regular heartbeats once initialized to stay invisible.
    if (vt.isInitialized && i != 15 &&
        (millis() - vt.lastHeartbeatTime > 900)) {
      vt.lastHeartbeatTime = millis();
      sendHeartbeat(i);
    }
  }

  // Parse RX packets from Server (PingPong, Handshakes) immediately to clear
  // buffers
  {
    // Parse ALL incoming packets for trackers to avoid LWIP buffer exhaustion
    for (int i = 0; i < 16; i++) {
      VirtualTracker &vt = _trackers[i];
      if (!vt.active)
        continue;

      while (int packetSize = vt.udp.parsePacket()) {
        uint8_t buffer[128];
        int len = vt.udp.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
          buffer[len] = '\0';

          // Diagnostic: Show where the packet came from
          if (vt.handshakeOngoing) {
            DEBUG_PRINTF("Got %d bytes from %s:%d on Tracker %d\n", len,
                         vt.udp.remoteIP().toString().c_str(),
                         vt.udp.remotePort(), i);
          }

          if (_discoveryMode) {
            _serverIp = vt.udp.remoteIP();
            _discoveryMode = false;
            _globalHandshakeSuccess = true;
            DEBUG_PRINT("[SlimeVR] Found Server at: ");
            DEBUG_PRINTLN(_serverIp);
            SerialManager::logToActivePorts(
                "Handshake successful"); // For SlimeVR Provisioning Wizard
          }

          vt.lastPacketReceivedTime = millis();

          // Packet structure:
          // [0-3] Packet Type (int32 BIG ENDIAN) - except handshake which is
          // [0] [4-11] Packet Number (int64 BIG ENDIAN)

          uint32_t packetType = 0;
          if (len >= 4) {
            packetType = (buffer[0] << 24) | (buffer[1] << 16) |
                         (buffer[2] << 8) | buffer[3];
          }

          // Case: PingPong (10) - Echo back to server
          if (packetType == 10) {
            vt.udp.beginPacket(vt.udp.remoteIP(), vt.udp.remotePort());
            vt.udp.write(buffer, len);
            vt.udp.endPacket();
            continue;
          }

          // Case: Heartbeat (1) - Reply with type 0
          if (packetType == 1) {
            sendHeartbeat(i);
            continue;
          }

          // Handshake detection: support both 4-byte BE (packetType == 3) and
          // 1-byte headers (buffer[0] == 3)
          const char *needle = "Hey OVR =D 5";
          int needleLen = 12;
          bool isHandshakeResponse = false;

          if (packetType == 3 || buffer[0] == 3) {
            for (int j = 0; j <= len - needleLen; j++) {
              if (memcmp(buffer + j, needle, needleLen) == 0) {
                isHandshakeResponse = true;
                break;
              }
            }
          }

          if (isHandshakeResponse) {
            if (vt.handshakeOngoing) {
              vt.handshakeOngoing = false;
              vt.isInitialized = true;
              vt.lastPacketReceivedTime = millis();

              // Tracker (15) doesn't need to show up in SlimeVR server.
              // We just use it to confirm the server connection and advance the
              // state machine.
              if (i == 15) {
                DEBUG_PRINTLN("[SlimeVR] Hub/System handshake successful "
                              "(Internal Only)");
                _globalHandshakeSuccess = true;
                vt.lastTimeoutTime = 0; // Reset cooldown on success
                vt.active = false;      // Deactivate the dummy tracker
                vt.udp.stop();          // Close the discovery socket
              } else {
                DEBUG_PRINTF(
                    "Handshake SUCCESS for Tracker %d - Sending confirmation\n",
                    i);
                vt.lastTimeoutTime = 0;        // Reset cooldown on success
                sendHandshake(i, vt.firmware); // Second handshake as required
                addTracker(i, vt.imuType, vt.firmware);
              }
            }
          }
        }
      }
    }
  }
}

long SlimeUdpClient::nextPacketId(uint8_t trackerIndex) {
  if (trackerIndex >= 16)
    return 0;
  return _trackers[trackerIndex].packetId++;
}

void SlimeUdpClient::sendHeartbeat(uint8_t trackerIndex) {
  if (trackerIndex >= 16 || !_trackers[trackerIndex].active)
    return;
  int offset = 0;

  // Header (int32)
  int packetType = 0; // HEARTBEAT
  _bufHeartbeat[offset++] = (packetType >> 24) & 0xFF;
  _bufHeartbeat[offset++] = (packetType >> 16) & 0xFF;
  _bufHeartbeat[offset++] = (packetType >> 8) & 0xFF;
  _bufHeartbeat[offset++] = packetType & 0xFF;

  // Packet counter (int64)
  long id = nextPacketId(trackerIndex);
  _bufHeartbeat[offset++] = ((uint64_t)id >> 56) & 0xFF;
  _bufHeartbeat[offset++] = ((uint64_t)id >> 48) & 0xFF;
  _bufHeartbeat[offset++] = ((uint64_t)id >> 40) & 0xFF;
  _bufHeartbeat[offset++] = ((uint64_t)id >> 32) & 0xFF;
  _bufHeartbeat[offset++] = ((uint64_t)id >> 24) & 0xFF;
  _bufHeartbeat[offset++] = ((uint64_t)id >> 16) & 0xFF;
  _bufHeartbeat[offset++] = ((uint64_t)id >> 8) & 0xFF;
  _bufHeartbeat[offset++] = id & 0xFF;

  // Tracker Id
  _bufHeartbeat[offset++] = 0;

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort))
    return;
  _trackers[trackerIndex].udp.write(_bufHeartbeat, offset);

  if (!_trackers[trackerIndex].udp.endPacket()) {
    static uint32_t lastErrorLog = 0;
    if (millis() - lastErrorLog > 10000) {
      SerialManager::logHeapStatus();
      lastErrorLog = millis();
    }
  }
}

void SlimeUdpClient::sendHandshake(uint8_t trackerIndex,
                                   const char *firmwareVersion) {
  if (trackerIndex >= 16 || !_trackers[trackerIndex].active)
    return;
  VirtualTracker &vt = _trackers[trackerIndex];
  int offset = 0;

  int packetType = 3; // HANDSHAKE
  _bufHandshake[offset++] = (packetType >> 24) & 0xFF;
  _bufHandshake[offset++] = (packetType >> 16) & 0xFF;
  _bufHandshake[offset++] = (packetType >> 8) & 0xFF;
  _bufHandshake[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  _bufHandshake[offset++] = ((uint64_t)id >> 56) & 0xFF;
  _bufHandshake[offset++] = ((uint64_t)id >> 48) & 0xFF;
  _bufHandshake[offset++] = ((uint64_t)id >> 40) & 0xFF;
  _bufHandshake[offset++] = ((uint64_t)id >> 32) & 0xFF;
  _bufHandshake[offset++] = ((uint64_t)id >> 24) & 0xFF;
  _bufHandshake[offset++] = ((uint64_t)id >> 16) & 0xFF;
  _bufHandshake[offset++] = ((uint64_t)id >> 8) & 0xFF;
  _bufHandshake[offset++] = id & 0xFF;

  // Board type, IMU type, etc.
  int boardType = vt.boardType;
  int imuType = 0; // UNKNOWN (defined in tracker registration later)
  int mcuType = vt.mcuType;
  int magStatus = 0;

  // Write Int32 x 6
  int values[] = {boardType, imuType,   mcuType,         magStatus,
                  magStatus, magStatus, _protocolVersion};
  for (int i = 0; i < 7; i++) {
    _bufHandshake[offset++] = (values[i] >> 24) & 0xFF;
    _bufHandshake[offset++] = (values[i] >> 16) & 0xFF;
    _bufHandshake[offset++] = (values[i] >> 8) & 0xFF;
    _bufHandshake[offset++] = values[i] & 0xFF;
  }

  if (firmwareVersion != nullptr) {
    size_t fwLen = strnlen(firmwareVersion, 31);
    _bufHandshake[offset++] = (uint8_t)fwLen;
    memcpy(_bufHandshake + offset, firmwareVersion, fwLen);
    offset += fwLen;
  } else {
    _bufHandshake[offset++] = 0; // zero length firmware string
  }

  for (int i = 0; i < 6; i++) {
    _bufHandshake[offset++] = vt.hardwareAddress[i];
  }

  if (_discoveryMode) {
    // Dual Broadcast: Send to both global and subnet-specific broadcast
    // addresses. This bypasses router restrictions on directed broadcast
    // packets.
    IPAddress subnetBroadcast = WiFi.broadcastIP();

    if (_trackers[trackerIndex].udp.beginPacket(IPAddress(255, 255, 255, 255),
                                                _serverPort)) {
      _trackers[trackerIndex].udp.write(_bufHandshake, offset);
      _trackers[trackerIndex].udp.endPacket();
    }

    if (subnetBroadcast != IPAddress(255, 255, 255, 255)) {
      if (_trackers[trackerIndex].udp.beginPacket(subnetBroadcast,
                                                  _serverPort)) {
        _trackers[trackerIndex].udp.write(_bufHandshake, offset);
        _trackers[trackerIndex].udp.endPacket();
      }
    }
  } else {
    // Sen directed handshake to known server.
    if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort))
      return;
    _trackers[trackerIndex].udp.write(_bufHandshake, offset);
    if (!_trackers[trackerIndex].udp.endPacket()) {
      static uint32_t lastErrorLog = 0;
      if (millis() - lastErrorLog > 10000) {
        SerialManager::logHeapStatus();
        lastErrorLog = millis();
      }
    }

    // Send twice but with a tiny gap to ensure the server stack can handle it.
    delayMicroseconds(500);
    if (_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort)) {
      _trackers[trackerIndex].udp.write(_bufHandshake, offset);
      _trackers[trackerIndex].udp.endPacket();
    }
  }
}

void SlimeUdpClient::addTracker(uint8_t trackerIndex, int imuType,
                                const char *firmwareVersion) {
  if (trackerIndex >= 16 || !_trackers[trackerIndex].active)
    return;
  int offset = 0;

  int packetType = 15; // SENSOR_INFO
  _bufSensorInfo[offset++] = (packetType >> 24) & 0xFF;
  _bufSensorInfo[offset++] = (packetType >> 16) & 0xFF;
  _bufSensorInfo[offset++] = (packetType >> 8) & 0xFF;
  _bufSensorInfo[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  _bufSensorInfo[offset++] = ((uint64_t)id >> 56) & 0xFF;
  _bufSensorInfo[offset++] = ((uint64_t)id >> 48) & 0xFF;
  _bufSensorInfo[offset++] = ((uint64_t)id >> 40) & 0xFF;
  _bufSensorInfo[offset++] = ((uint64_t)id >> 32) & 0xFF;
  _bufSensorInfo[offset++] = ((uint64_t)id >> 24) & 0xFF;
  _bufSensorInfo[offset++] = ((uint64_t)id >> 16) & 0xFF;
  _bufSensorInfo[offset++] = ((uint64_t)id >> 8) & 0xFF;
  _bufSensorInfo[offset++] = id & 0xFF;

  _bufSensorInfo[offset++] =
      0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension
  _bufSensorInfo[offset++] = 0;              // Sensor status (ok)
  _bufSensorInfo[offset++] = imuType & 0xFF; // IMU Type

  // Calibration State (int16)
  _bufSensorInfo[offset++] = 0;
  _bufSensorInfo[offset++] = 1;

  _bufSensorInfo[offset++] = 0; // Tracker Position (none)
  _bufSensorInfo[offset++] = 1; // Tracker Data Type (ROTATION)

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort))
    return;
  _trackers[trackerIndex].udp.write(_bufSensorInfo, offset);

  if (!_trackers[trackerIndex].udp.endPacket()) {
    static uint32_t lastErrorLog = 0;
    if (millis() - lastErrorLog > 10000) {
      SerialManager::logHeapStatus();
      lastErrorLog = millis();
    }
  }

  DEBUG_PRINTF("Registered Tracker Index: %d\n", trackerIndex);
}

void SlimeUdpClient::sendRotation(uint8_t trackerIndex, float qx, float qy,
                                  float qz, float qw) {
  if (trackerIndex >= 16 || !_trackers[trackerIndex].active ||
      !_trackers[trackerIndex].isInitialized)
    return;

  VirtualTracker &vt = _trackers[trackerIndex];

  // Rate Cap (Optimization)
  // Ensure we don't send data more than once every 4ms (Allows 100Hz Rotation +
  // 100Hz Accel interleaved)
  if (millis() - vt.lastMovementPacketTime < 4) {
    return;
  }

  // Movement Thresholding for Rotation (Dot product of 4D vectors)
  // If the change is small enough, and we sent a packet recently, skip this
  // one.
  float dot = (qx * vt.last_qx) + (qy * vt.last_qy) + (qz * vt.last_qz) +
              (qw * vt.last_qw);
  float diff = 1.0f - abs(dot);
  if (diff < MOVEMENT_THRESHOLD_QUAT &&
      (millis() - vt.lastMovementPacketTime < 500)) {
    return;
  }

  int offset = 0;

  int packetType = 17; // ROTATION_DATA
  _bufRotation[offset++] = (packetType >> 24) & 0xFF;
  _bufRotation[offset++] = (packetType >> 16) & 0xFF;
  _bufRotation[offset++] = (packetType >> 8) & 0xFF;
  _bufRotation[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  _bufRotation[offset++] = ((uint64_t)id >> 56) & 0xFF;
  _bufRotation[offset++] = ((uint64_t)id >> 48) & 0xFF;
  _bufRotation[offset++] = ((uint64_t)id >> 40) & 0xFF;
  _bufRotation[offset++] = ((uint64_t)id >> 32) & 0xFF;
  _bufRotation[offset++] = ((uint64_t)id >> 24) & 0xFF;
  _bufRotation[offset++] = ((uint64_t)id >> 16) & 0xFF;
  _bufRotation[offset++] = ((uint64_t)id >> 8) & 0xFF;
  _bufRotation[offset++] = id & 0xFF;

  _bufRotation[offset++] =
      0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension
  _bufRotation[offset++] = 1; // DataType

  // Floats in Big Endian
  uint32_t val;
  val = float_to_uint32(qx);
  _bufRotation[offset++] = (val >> 24) & 0xFF;
  _bufRotation[offset++] = (val >> 16) & 0xFF;
  _bufRotation[offset++] = (val >> 8) & 0xFF;
  _bufRotation[offset++] = val & 0xFF;
  val = float_to_uint32(qy);
  _bufRotation[offset++] = (val >> 24) & 0xFF;
  _bufRotation[offset++] = (val >> 16) & 0xFF;
  _bufRotation[offset++] = (val >> 8) & 0xFF;
  _bufRotation[offset++] = val & 0xFF;
  val = float_to_uint32(qz);
  _bufRotation[offset++] = (val >> 24) & 0xFF;
  _bufRotation[offset++] = (val >> 16) & 0xFF;
  _bufRotation[offset++] = (val >> 8) & 0xFF;
  _bufRotation[offset++] = val & 0xFF;
  val = float_to_uint32(qw);
  _bufRotation[offset++] = (val >> 24) & 0xFF;
  _bufRotation[offset++] = (val >> 16) & 0xFF;
  _bufRotation[offset++] = (val >> 8) & 0xFF;
  _bufRotation[offset++] = val & 0xFF;

  _bufRotation[offset++] = 0; // Calibration Info

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort))
    return;
  _trackers[trackerIndex].udp.write(_bufRotation, offset);

  if (!_trackers[trackerIndex].udp.endPacket()) {
    int errorCode = errno;
    if (errorCode == 12) { // ENOMEM
      static uint32_t lastErrorLog = 0;
      if (millis() - lastErrorLog > 10000) {
        DEBUG_PRINTLN(
            "[SlimeVR] Error 12 (ENOMEM) on Rotation Packet - TX Buffers Full");
        SerialManager::logHeapStatus(); // Log heap status ONLY within the
                                        // throttled block
        lastErrorLog = millis();
      }
    }
  } else {
    // Update last state only on successful send
    VirtualTracker &vt = _trackers[trackerIndex];
    vt.last_qx = qx;
    vt.last_qy = qy;
    vt.last_qz = qz;
    vt.last_qw = qw;
    vt.lastMovementPacketTime = millis();
  }
}

void SlimeUdpClient::sendAcceleration(uint8_t trackerIndex, float ax, float ay,
                                      float az) {
  if (trackerIndex >= 16 || !_trackers[trackerIndex].active ||
      !_trackers[trackerIndex].isInitialized)
    return;

  VirtualTracker &vt = _trackers[trackerIndex];

  // Rate Cap (Optimization)
  if (millis() - vt.lastMovementPacketTime < 4) {
    return;
  }

  // Movement Thresholding for Acceleration (Euclidean distance)
  float dx = ax - vt.last_ax;
  float dy = ay - vt.last_ay;
  float dz = az - vt.last_az;
  float distSq = (dx * dx) + (dy * dy) + (dz * dz);
  if (distSq < (MOVEMENT_THRESHOLD_ACCEL * MOVEMENT_THRESHOLD_ACCEL) &&
      (millis() - vt.lastMovementPacketTime < 500)) {
    return;
  }

  int offset = 0;

  int packetType = 4; // ACCELERATION
  _bufAcceleration[offset++] = (packetType >> 24) & 0xFF;
  _bufAcceleration[offset++] = (packetType >> 16) & 0xFF;
  _bufAcceleration[offset++] = (packetType >> 8) & 0xFF;
  _bufAcceleration[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  _bufAcceleration[offset++] = ((uint64_t)id >> 56) & 0xFF;
  _bufAcceleration[offset++] = ((uint64_t)id >> 48) & 0xFF;
  _bufAcceleration[offset++] = ((uint64_t)id >> 40) & 0xFF;
  _bufAcceleration[offset++] = ((uint64_t)id >> 32) & 0xFF;
  _bufAcceleration[offset++] = ((uint64_t)id >> 24) & 0xFF;
  _bufAcceleration[offset++] = ((uint64_t)id >> 16) & 0xFF;
  _bufAcceleration[offset++] = ((uint64_t)id >> 8) & 0xFF;
  _bufAcceleration[offset++] = id & 0xFF;

  uint32_t val;
  val = float_to_uint32(ax);
  _bufAcceleration[offset++] = (val >> 24) & 0xFF;
  _bufAcceleration[offset++] = (val >> 16) & 0xFF;
  _bufAcceleration[offset++] = (val >> 8) & 0xFF;
  _bufAcceleration[offset++] = val & 0xFF;
  val = float_to_uint32(ay);
  _bufAcceleration[offset++] = (val >> 24) & 0xFF;
  _bufAcceleration[offset++] = (val >> 16) & 0xFF;
  _bufAcceleration[offset++] = (val >> 8) & 0xFF;
  _bufAcceleration[offset++] = val & 0xFF;
  val = float_to_uint32(az);
  _bufAcceleration[offset++] = (val >> 24) & 0xFF;
  _bufAcceleration[offset++] = (val >> 16) & 0xFF;
  _bufAcceleration[offset++] = (val >> 8) & 0xFF;
  _bufAcceleration[offset++] = val & 0xFF;

  _bufAcceleration[offset++] =
      0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort))
    return;
  _trackers[trackerIndex].udp.write(_bufAcceleration, offset);

  if (!_trackers[trackerIndex].udp.endPacket()) {
    int errorCode = errno;
    if (errorCode == 12) { // ENOMEM
      static uint32_t lastErrorLog = 0;
      if (millis() - lastErrorLog > 10000) {
        DEBUG_PRINTLN("[SlimeVR] Error 12 (ENOMEM) on Acceleration Packet - TX "
                      "Buffers Full");
        SerialManager::logHeapStatus(); // Log heap status ONLY within the
                                        // throttled block
        lastErrorLog = millis();
      }
    }
  } else {
    // Update last state only on successful send
    VirtualTracker &vt = _trackers[trackerIndex];
    vt.last_ax = ax;
    vt.last_ay = ay;
    vt.last_az = az;
    vt.lastMovementPacketTime = millis();
  }
}

void SlimeUdpClient::sendBattery(uint8_t trackerIndex, float voltage,
                                 float batteryPercentage) {
  if (trackerIndex >= 16 || !_trackers[trackerIndex].active ||
      !_trackers[trackerIndex].isInitialized)
    return;
  int offset = 0;

  int packetType = 12; // BATTERY_LEVEL
  _bufBattery[offset++] = (packetType >> 24) & 0xFF;
  _bufBattery[offset++] = (packetType >> 16) & 0xFF;
  _bufBattery[offset++] = (packetType >> 8) & 0xFF;
  _bufBattery[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  _bufBattery[offset++] = ((uint64_t)id >> 56) & 0xFF;
  _bufBattery[offset++] = ((uint64_t)id >> 48) & 0xFF;
  _bufBattery[offset++] = ((uint64_t)id >> 40) & 0xFF;
  _bufBattery[offset++] = ((uint64_t)id >> 32) & 0xFF;
  _bufBattery[offset++] = ((uint64_t)id >> 24) & 0xFF;
  _bufBattery[offset++] = ((uint64_t)id >> 16) & 0xFF;
  _bufBattery[offset++] = ((uint64_t)id >> 8) & 0xFF;
  _bufBattery[offset++] = id & 0xFF;

  uint32_t val;
  val = float_to_uint32(voltage);
  _bufBattery[offset++] = (val >> 24) & 0xFF;
  _bufBattery[offset++] = (val >> 16) & 0xFF;
  _bufBattery[offset++] = (val >> 8) & 0xFF;
  _bufBattery[offset++] = val & 0xFF;

  // Percentage needs to be fraction
  float pct = batteryPercentage / 100.0f;
  val = float_to_uint32(pct);
  _bufBattery[offset++] = (val >> 24) & 0xFF;
  _bufBattery[offset++] = (val >> 16) & 0xFF;
  _bufBattery[offset++] = (val >> 8) & 0xFF;
  _bufBattery[offset++] = val & 0xFF;

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort))
    return;
  _trackers[trackerIndex].udp.write(_bufBattery, offset);

  if (!_trackers[trackerIndex].udp.endPacket()) {
    static uint32_t lastErrorLog = 0;
    if (millis() - lastErrorLog > 10000) {
      SerialManager::logHeapStatus();
      lastErrorLog = millis();
    }
  }
}
