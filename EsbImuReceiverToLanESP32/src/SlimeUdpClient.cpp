#include "SlimeUdpClient.h"
#include "config.h"
#include <vector>
#include <string.h>

SlimeUdpClient::SlimeUdpClient() {
    _protocolVersion = 19;
    _serverPort = 6969;
}

void SlimeUdpClient::begin(const char* serverIp, uint16_t serverPort) {
    _serverIp.fromString(serverIp);
    _serverPort = serverPort;
    
    // Trackers will initialize their own sockets when registered
    
    DEBUG_PRINTLN("SlimeUdpClient Initialized");
}

void SlimeUdpClient::initializeTracker(uint8_t trackerIndex, const uint8_t mac[6], int imuType) {
    if (trackerIndex >= 40) return;
    VirtualTracker& vt = _trackers[trackerIndex];
    if (vt.active) return;

    vt.active = true;
    memcpy(vt.hardwareAddress, mac, 6);
    vt.packetId = 0;
    vt.handshakeOngoing = true;
    vt.isInitialized = false;
    vt.imuType = imuType;
    vt.lastHeartbeatTime = millis();
    vt.lastHandshakeTime = millis();
    vt.udp.begin(50000 + trackerIndex); // Bind to a unique local ephemeral port
    
    sendHandshake(trackerIndex);
}

void SlimeUdpClient::loop() {
    for (int i = 0; i < 40; i++) {
        VirtualTracker& vt = _trackers[i];
        if (!vt.active) continue;

        if (vt.handshakeOngoing) {
            // Check handshake response, retry every 1000ms
            if (millis() - vt.lastHandshakeTime > 1000) {
                vt.lastHandshakeTime = millis();
                sendHandshake(i);
            }
        }

        if (millis() - vt.lastHeartbeatTime > 900) {
            vt.lastHeartbeatTime = millis();
            sendHeartbeat(i);
        }

        // Parse incoming packets for this specific tracker
        int packetSize = vt.udp.parsePacket();
        if (packetSize) {
            uint8_t buffer[128];
            int len = vt.udp.read(buffer, sizeof(buffer));
            if (len > 0) {
                // Check for Hey OVR =D 5 string to confirm handshake
                String response((char*)buffer);
                if (response.indexOf("Hey OVR =D 5") >= 0) {
                    if (vt.handshakeOngoing) {
                        vt.handshakeOngoing = false;
                        vt.isInitialized = true;
                        
                        // C# App sends one last Handshake immediately, followed by the SENSOR_INFO
                        sendHandshake(i);
                        addTracker(i, vt.imuType);

#if ENABLE_DEBUG_PRINT
                        DEBUG_PRINTF("SlimeVR Handshake Successful for Tracker %d!\n", i);
#endif
                    }
                }
            }
        }
    }
}

long SlimeUdpClient::nextPacketId(uint8_t trackerIndex) {
    if (trackerIndex >= 40) return 0;
    return _trackers[trackerIndex].packetId++;
}

void SlimeUdpClient::sendHeartbeat(uint8_t trackerIndex) {
    if (trackerIndex >= 40) return;
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

    _trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort);
    _trackers[trackerIndex].udp.write(buffer, offset);
    _trackers[trackerIndex].udp.endPacket();

    DEBUG_PRINTF("Sent Heartbeat for Tracker %d\n", trackerIndex);
}

void SlimeUdpClient::sendHandshake(uint8_t trackerIndex) {
    if (trackerIndex >= 40) return;
    VirtualTracker& vt = _trackers[trackerIndex];
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
    int boardType = 0; // CUSTOM
    int imuType = 0; // UNKNOWN (defined in tracker registration later)
    int mcuType = 2; // ESP32
    int magStatus = 0; 
    
    // Write Int32 x 6
    int values[] = {boardType, imuType, mcuType, magStatus, magStatus, magStatus, _protocolVersion};
    for (int i=0; i<7; i++) {
        buffer[offset++] = (values[i] >> 24) & 0xFF;
        buffer[offset++] = (values[i] >> 16) & 0xFF;
        buffer[offset++] = (values[i] >> 8) & 0xFF;
        buffer[offset++] = values[i] & 0xFF;
    }

    String fwString = "Bootleg Tracker ESB";
    buffer[offset++] = fwString.length();
    for (int i=0; i<fwString.length(); i++) {
        buffer[offset++] = fwString[i];
    }

    for (int i=0; i<6; i++) {
        buffer[offset++] = vt.hardwareAddress[i];
    }
    
    _trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort);
    _trackers[trackerIndex].udp.write(buffer, offset);
    _trackers[trackerIndex].udp.endPacket();

    DEBUG_PRINTF("Sent Handshake to SlimeVR for Tracker %d!\n", trackerIndex);
}

void SlimeUdpClient::addTracker(uint8_t trackerIndex, int imuType) {
    if (trackerIndex >= 40) return;
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

    buffer[offset++] = 0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension
    buffer[offset++] = 0; // Sensor status (ok)
    buffer[offset++] = imuType & 0xFF; // IMU Type
    
    // Calibration State (int16)
    buffer[offset++] = 0;
    buffer[offset++] = 1;

    buffer[offset++] = 0; // Tracker Position (none)
    buffer[offset++] = 1; // Tracker Data Type (ROTATION)

    _trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort);
    _trackers[trackerIndex].udp.write(buffer, offset);
    _trackers[trackerIndex].udp.endPacket();

    DEBUG_PRINTF("Registered Tracker Index: %d\n", trackerIndex);
}

void SlimeUdpClient::sendRotation(uint8_t trackerIndex, float qx, float qy, float qz, float qw) {
    if (trackerIndex >= 40) return;
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

    buffer[offset++] = 0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension
    buffer[offset++] = 1; // DataType

    // Floats in Big Endian
    uint32_t val;
    memcpy(&val, &qx, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    memcpy(&val, &qy, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    memcpy(&val, &qz, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    memcpy(&val, &qw, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;

    buffer[offset++] = 0; // Calibration Info

    _trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort);
    _trackers[trackerIndex].udp.write(buffer, offset);
    _trackers[trackerIndex].udp.endPacket();
}

void SlimeUdpClient::sendAcceleration(uint8_t trackerIndex, float ax, float ay, float az) {
    if (trackerIndex >= 40) return;
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
    memcpy(&val, &ax, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    memcpy(&val, &ay, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    memcpy(&val, &az, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;

    buffer[offset++] = 0; // Hardcoded Tracker ID 0 so it's a main tracker, not an extension

    _trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort);
    _trackers[trackerIndex].udp.write(buffer, offset);
    _trackers[trackerIndex].udp.endPacket();
}

void SlimeUdpClient::sendBattery(uint8_t trackerIndex, float voltage, float batteryPercentage) {
    if (trackerIndex >= 40) return;
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
    memcpy(&val, &voltage, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    
    // Percentage needs to be fraction
    float pct = batteryPercentage / 100.0f;
    memcpy(&val, &pct, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;

    _trackers[trackerIndex].udp.beginPacket(_serverIp, _serverPort);
    _trackers[trackerIndex].udp.write(buffer, offset);
    _trackers[trackerIndex].udp.endPacket();
}
