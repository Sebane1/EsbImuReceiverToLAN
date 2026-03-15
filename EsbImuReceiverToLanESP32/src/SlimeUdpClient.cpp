#include "SlimeUdpClient.h"
#include "config.h"
#include <string.h>
#include <vector>

// Safe float-to-uint32 bitcast using union (avoids Xtensa alignment crash from memcpy on register-resident floats)
static inline uint32_t float_to_uint32(float f) {
  union { float f; uint32_t u; } conv;
  conv.f = f;
  return conv.u;
}

SlimeUdpClient::SlimeUdpClient() {
  _protocolVersion = 22;
  _serverPort = 6969;
}

void SlimeUdpClient::begin(const char *serverIp, uint16_t serverPort) {
  _serverIp.fromString(serverIp);
  _serverPort = serverPort;
  randomSeed(micros()); // Seed randomness for dynamic port selection
  DEBUG_PRINTLN("SlimeUdpClient Initialized");
}

void SlimeUdpClient::onWiFiDisconnect() {
  for (int i = 0; i < 11; i++) {
    if (_trackers[i].active) {
      _trackers[i].udp.stop();
    }
  }
}

void SlimeUdpClient::onWiFiConnect() {
  for (int i = 0; i < 11; i++) {
    // Dynamic Port Randomization: Use random ephemeral ports to bypass "stuck" server states.
    if (_trackers[i].active && i < 11) {
      _trackers[i].lastSendDataTime = 0;
      _trackers[i].lastBatterySendTime = 0;
      _trackers[i].lastErrMemTime = 0;
      _trackers[i].handshakeRetryCount = 0;
      _trackers[i].imuType = 0;
      uint16_t randomPort = 50000 + random(0, 10000);
      _trackers[i].udp.begin(randomPort); 
      _trackers[i].handshakeOngoing = true; // Force handshake on reconnect
      _trackers[i].isInitialized = false;
    }
  }
}

void SlimeUdpClient::initializeTracker(uint8_t trackerIndex,
                                       const uint8_t mac[6], int imuType, int boardType, int mcuType,
                                       const char *firmwareVersion) {
  if (trackerIndex >= 11)
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

  // ONLY bind the socket if we are under the hardware limit of ~16 sockets
  if (trackerIndex < 11) {
      uint16_t randomPort = 50000 + random(0, 10000);
      vt.udp.begin(randomPort);
  }

  sendHandshake(trackerIndex, vt.firmware);
}

void SlimeUdpClient::loop() {
  for (int i = 0; i < 11; i++) {
    VirtualTracker &vt = _trackers[i];
    if (!vt.active)
      continue;

    if (vt.handshakeOngoing) {
      // Sequential Handshake: Only one tracker handshakes at a time.
      // A tracker only attempts its handshake if all active trackers with a lower index are already initialized.
      bool lowerTrackersPending = false;
      for (int prev = 0; prev < i; prev++) {
          if (_trackers[prev].active && !_trackers[prev].isInitialized) {
              lowerTrackersPending = true;
              break;
          }
      }

      if (!lowerTrackersPending) {
        // Stagger handshakes: 500ms + trackerIndex * 50ms
        // Increased aggression to ensure SlimeVR server picks up the connection.
        if (millis() - vt.lastHandshakeTime > (500 + (i * 50))) {
          if (isNetworkReady(i)) { // Don't send if network is congested
              vt.lastHandshakeTime = millis();
              vt.handshakeRetryCount++;
              
              // If we've retried 10 times without success, reset the socket and LEAP to a new random port.
              // This clears potential "stuck" states and bypasses port-specific blocks.
              if (vt.handshakeRetryCount % 10 == 0) {
                  uint16_t newPort = 50000 + random(0, 10000);
                  DEBUG_PRINTF("Resetting socket for Tracker %d (New Port: %d) after %d retries\n", i, newPort, vt.handshakeRetryCount);
                  vt.udp.stop();
                  vt.udp.begin(newPort);
              }
              
              sendHandshake(i, vt.firmware);
          }
        }
      }
    }

    if (millis() - vt.lastHeartbeatTime > 900) {
      if (isNetworkReady(i)) {
          vt.lastHeartbeatTime = millis();
          sendHeartbeat(i);
      }
    }
  }

  // Parse RX packets from Server (PingPong, Handshakes) immediately to clear buffers
  // Removed 10ms throttle to avoid backlogged replies
  {
    // Parse ALL incoming packets for trackers to avoid LWIP buffer exhaustion
    for (int i = 0; i < 11; i++) {
      VirtualTracker &vt = _trackers[i];
      if (!vt.active) continue;

      while (int packetSize = vt.udp.parsePacket()) {
        uint8_t buffer[128];
        int len = vt.udp.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
          buffer[len] = '\0';
          
          // Diagnostic: Show where the packet came from
          if (vt.handshakeOngoing) {
              DEBUG_PRINTF("Got %d bytes from %s:%d on Tracker %d\n", len, vt.udp.remoteIP().toString().c_str(), vt.udp.remotePort(), i);
          }
          
          // For Handshakes, SlimeVR server responds with "Hey OVR =D 5"
          // Use raw memory search instead of String to avoid strlen() crash in ROM
          const char* needle = "Hey OVR =D 5";
          int needleLen = 12;
          bool found = false;
          for (int j = 0; j <= len - needleLen; j++) {
            if (memcmp(buffer + j, needle, needleLen) == 0) {
              found = true;
              break;
            }
          }
          if (found) {
             if (vt.handshakeOngoing) {
                vt.handshakeOngoing = false;
                vt.isInitialized = true;
                DEBUG_PRINTF("Handshake SUCCESS for Tracker %d - Sending confirmation\n", i);
                sendHandshake(i, vt.firmware); // Second handshake as required
                addTracker(i, vt.imuType, vt.firmware);
             }
          }
        }
      }
    }
  }
}

bool SlimeUdpClient::isNetworkReady(uint8_t trackerIndex) {
  if (trackerIndex >= 11 || !_trackers[trackerIndex].active) return false;
  
  uint32_t lastErr = _trackers[trackerIndex].lastErrMemTime;
  if (lastErr == 0) return true; // No errors yet, always ready
  
  // If we had a memory error recently, back off for 50ms to let buffers drain.
  if (millis() - lastErr < 50) {
    return false;
  }
  return true;
}

long SlimeUdpClient::nextPacketId(uint8_t trackerIndex) {
  if (trackerIndex >= 11)
    return 0;
  return _trackers[trackerIndex].packetId++;
}

void SlimeUdpClient::sendHeartbeat(uint8_t trackerIndex) {
  if (trackerIndex >= 11 || !_trackers[trackerIndex].active)
    return;
  uint8_t buffer[13];
  int offset = 0;

  // Header (int32)
  int packetType = 0; // HEARTBEAT
  buffer[offset++] = (packetType >> 24) & 0xFF;
  buffer[offset++] = (packetType >> 16) & 0xFF;
  buffer[offset++] = (packetType >> 8) & 0xFF;
  buffer[offset++] = packetType & 0xFF;

  // Packet counter (int64)
  long id = nextPacketId(trackerIndex);
  buffer[offset++] = ((uint64_t)id >> 56) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 48) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 40) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 32) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 24) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 16) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 8) & 0xFF;
  buffer[offset++] = id & 0xFF;

  // Tracker Id
  buffer[offset++] = 0;

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort)) return;
  _trackers[trackerIndex].udp.write(buffer, offset);
  if (!_trackers[trackerIndex].udp.endPacket()) {
    _trackers[trackerIndex].lastErrMemTime = millis();
  }
}

void SlimeUdpClient::sendHandshake(uint8_t trackerIndex,
                                   const char *firmwareVersion) {
  if (trackerIndex >= 11 || !_trackers[trackerIndex].active)
    return;
  VirtualTracker &vt = _trackers[trackerIndex];
  uint8_t buffer[256];
  int offset = 0;

  int packetType = 3; // HANDSHAKE
  buffer[offset++] = (packetType >> 24) & 0xFF;
  buffer[offset++] = (packetType >> 16) & 0xFF;
  buffer[offset++] = (packetType >> 8) & 0xFF;
  buffer[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  buffer[offset++] = ((uint64_t)id >> 56) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 48) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 40) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 32) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 24) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 16) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 8) & 0xFF;
  buffer[offset++] = id & 0xFF;

  // Board type, IMU type, etc.
  int boardType = vt.boardType;
  int imuType = 0;   // UNKNOWN (defined in tracker registration later)
  int mcuType = vt.mcuType;
  int magStatus = 0;

  // Write Int32 x 6
  int values[] = {boardType, imuType,   mcuType,         magStatus,
                  magStatus, magStatus, _protocolVersion};
  for (int i = 0; i < 7; i++) {
    buffer[offset++] = (values[i] >> 24) & 0xFF;
    buffer[offset++] = (values[i] >> 16) & 0xFF;
    buffer[offset++] = (values[i] >> 8) & 0xFF;
    buffer[offset++] = values[i] & 0xFF;
  }

  // Write firmware version string safely without String/strlen
  // vt.firmware is a char[32] with guaranteed null termination
  if (firmwareVersion != nullptr) {
    size_t fwLen = strnlen(firmwareVersion, 31);
    buffer[offset++] = (uint8_t)fwLen;
    memcpy(buffer + offset, firmwareVersion, fwLen);
    offset += fwLen;
  } else {
    buffer[offset++] = 0; // zero length firmware string
  }

  for (int i = 0; i < 6; i++) {
    buffer[offset++] = vt.hardwareAddress[i];
  }

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort)) return;
  _trackers[trackerIndex].udp.write(buffer, offset);
  _trackers[trackerIndex].udp.endPacket();
  
  // Burst: Send twice but with a tiny gap to ensure the server stack can handle it.
  delayMicroseconds(500);
  
  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort)) return;
  _trackers[trackerIndex].udp.write(buffer, offset);
  if (!_trackers[trackerIndex].udp.endPacket()) {
    _trackers[trackerIndex].lastErrMemTime = millis();
    DEBUG_PRINTF("Handshake burst FAILED (ERR_MEM) for Tracker %d\n", trackerIndex);
  } else {
    DEBUG_PRINTF("Sent Handshake (Burst) to SlimeVR for Tracker %d!\n", trackerIndex);
  }
}

void SlimeUdpClient::addTracker(uint8_t trackerIndex, int imuType,
                                const char *firmwareVersion) {
  if (trackerIndex >= 11 || !_trackers[trackerIndex].active)
    return;
  uint8_t buffer[20];
  int offset = 0;

  int packetType = 15; // SENSOR_INFO
  buffer[offset++] = (packetType >> 24) & 0xFF;
  buffer[offset++] = (packetType >> 16) & 0xFF;
  buffer[offset++] = (packetType >> 8) & 0xFF;
  buffer[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  buffer[offset++] = ((uint64_t)id >> 56) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 48) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 40) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 32) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 24) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 16) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 8) & 0xFF;
  buffer[offset++] = id & 0xFF;

  buffer[offset++] =
      0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension
  buffer[offset++] = 0;              // Sensor status (ok)
  buffer[offset++] = imuType & 0xFF; // IMU Type

  // Calibration State (int16)
  buffer[offset++] = 0;
  buffer[offset++] = 1;

  buffer[offset++] = 0; // Tracker Position (none)
  buffer[offset++] = 1; // Tracker Data Type (ROTATION)

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort)) return;
  _trackers[trackerIndex].udp.write(buffer, offset);
  if (!_trackers[trackerIndex].udp.endPacket()) {
    _trackers[trackerIndex].lastErrMemTime = millis();
  }

  DEBUG_PRINTF("Registered Tracker Index: %d\n", trackerIndex);
}

void SlimeUdpClient::sendRotation(uint8_t trackerIndex, float qx, float qy,
                                  float qz, float qw) {
  if (trackerIndex >= 11 || !_trackers[trackerIndex].active || !_trackers[trackerIndex].isInitialized)
    return;
  uint8_t buffer[35];
  int offset = 0;

  int packetType = 17; // ROTATION_DATA
  buffer[offset++] = (packetType >> 24) & 0xFF;
  buffer[offset++] = (packetType >> 16) & 0xFF;
  buffer[offset++] = (packetType >> 8) & 0xFF;
  buffer[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  buffer[offset++] = ((uint64_t)id >> 56) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 48) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 40) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 32) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 24) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 16) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 8) & 0xFF;
  buffer[offset++] = id & 0xFF;

  buffer[offset++] =
      0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension
  buffer[offset++] = 1; // DataType

  // Floats in Big Endian
  uint32_t val;
  val = float_to_uint32(qx);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;
  val = float_to_uint32(qy);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;
  val = float_to_uint32(qz);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;
  val = float_to_uint32(qw);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;

  buffer[offset++] = 0; // Calibration Info

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort)) return;
  _trackers[trackerIndex].udp.write(buffer, offset);
  if (!_trackers[trackerIndex].udp.endPacket()) {
    _trackers[trackerIndex].lastErrMemTime = millis();
  }
}

void SlimeUdpClient::sendAcceleration(uint8_t trackerIndex, float ax, float ay,
                                      float az) {
  if (trackerIndex >= 11 || !_trackers[trackerIndex].active || !_trackers[trackerIndex].isInitialized)
    return;
  uint8_t buffer[30];
  int offset = 0;

  int packetType = 4; // ACCELERATION
  buffer[offset++] = (packetType >> 24) & 0xFF;
  buffer[offset++] = (packetType >> 16) & 0xFF;
  buffer[offset++] = (packetType >> 8) & 0xFF;
  buffer[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  buffer[offset++] = ((uint64_t)id >> 56) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 48) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 40) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 32) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 24) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 16) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 8) & 0xFF;
  buffer[offset++] = id & 0xFF;

  uint32_t val;
  val = float_to_uint32(ax);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;
  val = float_to_uint32(ay);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;
  val = float_to_uint32(az);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;

  buffer[offset++] =
      0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort)) return;
  _trackers[trackerIndex].udp.write(buffer, offset);
  if (!_trackers[trackerIndex].udp.endPacket()) {
    _trackers[trackerIndex].lastErrMemTime = millis();
  }
}

void SlimeUdpClient::sendBattery(uint8_t trackerIndex, float voltage,
                                 float batteryPercentage) {
  if (trackerIndex >= 11 || !_trackers[trackerIndex].active || !_trackers[trackerIndex].isInitialized)
    return;
  uint8_t buffer[24];
  int offset = 0;

  int packetType = 12; // BATTERY_LEVEL
  buffer[offset++] = (packetType >> 24) & 0xFF;
  buffer[offset++] = (packetType >> 16) & 0xFF;
  buffer[offset++] = (packetType >> 8) & 0xFF;
  buffer[offset++] = packetType & 0xFF;

  long id = nextPacketId(trackerIndex);
  buffer[offset++] = ((uint64_t)id >> 56) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 48) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 40) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 32) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 24) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 16) & 0xFF;
  buffer[offset++] = ((uint64_t)id >> 8) & 0xFF;
  buffer[offset++] = id & 0xFF;

  uint32_t val;
  val = float_to_uint32(voltage);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;

  // Percentage needs to be fraction
  float pct = batteryPercentage / 100.0f;
  val = float_to_uint32(pct);
  buffer[offset++] = (val >> 24) & 0xFF;
  buffer[offset++] = (val >> 16) & 0xFF;
  buffer[offset++] = (val >> 8) & 0xFF;
  buffer[offset++] = val & 0xFF;

  if (!_trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort)) return;
  _trackers[trackerIndex].udp.write(buffer, offset);
  if (!_trackers[trackerIndex].udp.endPacket()) {
    _trackers[trackerIndex].lastErrMemTime = millis();
  }
}
