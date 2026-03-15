#include "SlimeUdpClient.h"
#include <vector>
#include <string.h>

SlimeUdpClient::SlimeUdpClient() {
    _packetId = 0;
    _protocolVersion = 19;
    _handshakeOngoing = false;
    _isInitialized = false;
    _serverPort = 6969;
    _lastHeartbeatTime = 0;
}

void SlimeUdpClient::begin(const char* ip, uint16_t port) {
    _serverIp.fromString(ip);
    _serverPort = port;
    _udp.begin(port);
}

void SlimeUdpClient::setHardwareAddress(uint8_t mac[6]) {
    memcpy(_hardwareAddress, mac, 6);
}

void SlimeUdpClient::forceHandshake() {
    _handshakeOngoing = true;
    _isInitialized = false;
    sendHandshake();
}

void SlimeUdpClient::loop() {
    if (_handshakeOngoing) {
        // Simple retry logic later, for now just send heartbeat
    }

    if (millis() - _lastHeartbeatTime > 900) {
        _lastHeartbeatTime = millis();
        sendHeartbeat();
    }

    // Need to parse incoming packets here if necessary (like heartbeat responses)
    int packetSize = _udp.parsePacket();
    if (packetSize) {
        uint8_t buffer[128];
        int len = _udp.read(buffer, sizeof(buffer));
        if (len > 0) {
            // Check for Hey OVR =D 5 string to confirm handshake
            String response((char*)buffer);
            if (response.indexOf("Hey OVR =D 5") >= 0) {
                _handshakeOngoing = false;
                _isInitialized = true;
                DEBUG_PRINTLN("SlimeVR Handshake Successful!");
            }
        }
    }
}

long SlimeUdpClient::nextPacketId() {
    return _packetId++;
}

void SlimeUdpClient::sendHeartbeat() {
    uint8_t buffer[13];
    int offset = 0;

    // Header (int32)
    int packetType = 0; // HEARTBEAT
    buffer[offset++] = (packetType >> 24) & 0xFF;
    buffer[offset++] = (packetType >> 16) & 0xFF;
    buffer[offset++] = (packetType >> 8) & 0xFF;
    buffer[offset++] = packetType & 0xFF;

    // Packet counter (int64)
    long id = nextPacketId();
    buffer[offset++] = (id >> 56) & 0xFF;
    buffer[offset++] = (id >> 48) & 0xFF;
    buffer[offset++] = (id >> 40) & 0xFF;
    buffer[offset++] = (id >> 32) & 0xFF;
    buffer[offset++] = (id >> 24) & 0xFF;
    buffer[offset++] = (id >> 16) & 0xFF;
    buffer[offset++] = (id >> 8) & 0xFF;
    buffer[offset++] = id & 0xFF;

    // Tracker Id
    buffer[offset++] = 0;

    _udp.beginPacket(_serverIp, _serverPort);
    _udp.write(buffer, offset);
    _udp.endPacket();
}

void SlimeUdpClient::sendHandshake() {
    uint8_t buffer[256];
    int offset = 0;
    
    int packetType = 3; // HANDSHAKE
    buffer[offset++] = (packetType >> 24) & 0xFF;
    buffer[offset++] = (packetType >> 16) & 0xFF;
    buffer[offset++] = (packetType >> 8) & 0xFF;
    buffer[offset++] = packetType & 0xFF;

    long id = nextPacketId();
    buffer[offset++] = (id >> 56) & 0xFF;
    buffer[offset++] = (id >> 48) & 0xFF;
    buffer[offset++] = (id >> 40) & 0xFF;
    buffer[offset++] = (id >> 32) & 0xFF;
    buffer[offset++] = (id >> 24) & 0xFF;
    buffer[offset++] = (id >> 16) & 0xFF;
    buffer[offset++] = (id >> 8) & 0xFF;
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
        buffer[offset++] = _hardwareAddress[i];
    }
    
    _udp.beginPacket(_serverIp, _serverPort);
    _udp.write(buffer, offset);
    _udp.endPacket();

    DEBUG_PRINTLN("Sent Handshake to SlimeVR!");
}

void SlimeUdpClient::addTracker(uint8_t trackerId, int imuType) {
    uint8_t buffer[20];
    int offset = 0;

    int packetType = 15; // SENSOR_INFO
    buffer[offset++] = (packetType >> 24) & 0xFF;
    buffer[offset++] = (packetType >> 16) & 0xFF;
    buffer[offset++] = (packetType >> 8) & 0xFF;
    buffer[offset++] = packetType & 0xFF;

    long id = nextPacketId();
    buffer[offset++] = (id >> 56) & 0xFF;
    buffer[offset++] = (id >> 48) & 0xFF;
    buffer[offset++] = (id >> 40) & 0xFF;
    buffer[offset++] = (id >> 32) & 0xFF;
    buffer[offset++] = (id >> 24) & 0xFF;
    buffer[offset++] = (id >> 16) & 0xFF;
    buffer[offset++] = (id >> 8) & 0xFF;
    buffer[offset++] = id & 0xFF;

    buffer[offset++] = trackerId;
    buffer[offset++] = 0; // Sensor status (ok)
    buffer[offset++] = imuType & 0xFF; // IMU Type
    
    // Calibration State (int16)
    buffer[offset++] = 0;
    buffer[offset++] = 1;

    buffer[offset++] = 0; // Tracker Position (none)
    buffer[offset++] = 1; // Tracker Data Type (ROTATION)

    _udp.beginPacket(_serverIp, _serverPort);
    _udp.write(buffer, offset);
    _udp.endPacket();

    DEBUG_PRINTF("Registered Tracker ID: %d\n", trackerId);
}

void SlimeUdpClient::sendRotation(uint8_t trackerId, float qx, float qy, float qz, float qw) {
    uint8_t buffer[35];
    int offset = 0;

    int packetType = 17; // ROTATION_DATA
    buffer[offset++] = (packetType >> 24) & 0xFF;
    buffer[offset++] = (packetType >> 16) & 0xFF;
    buffer[offset++] = (packetType >> 8) & 0xFF;
    buffer[offset++] = packetType & 0xFF;

    long id = nextPacketId();
    buffer[offset++] = (id >> 56) & 0xFF;
    buffer[offset++] = (id >> 48) & 0xFF;
    buffer[offset++] = (id >> 40) & 0xFF;
    buffer[offset++] = (id >> 32) & 0xFF;
    buffer[offset++] = (id >> 24) & 0xFF;
    buffer[offset++] = (id >> 16) & 0xFF;
    buffer[offset++] = (id >> 8) & 0xFF;
    buffer[offset++] = id & 0xFF;

    buffer[offset++] = trackerId;
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

    _udp.beginPacket(_serverIp, _serverPort);
    _udp.write(buffer, offset);
    _udp.endPacket();
}

void SlimeUdpClient::sendAcceleration(uint8_t trackerId, float ax, float ay, float az) {
    uint8_t buffer[30];
    int offset = 0;

    int packetType = 4; // ACCELERATION
    buffer[offset++] = (packetType >> 24) & 0xFF;
    buffer[offset++] = (packetType >> 16) & 0xFF;
    buffer[offset++] = (packetType >> 8) & 0xFF;
    buffer[offset++] = packetType & 0xFF;

    long id = nextPacketId();
    buffer[offset++] = (id >> 56) & 0xFF;
    buffer[offset++] = (id >> 48) & 0xFF;
    buffer[offset++] = (id >> 40) & 0xFF;
    buffer[offset++] = (id >> 32) & 0xFF;
    buffer[offset++] = (id >> 24) & 0xFF;
    buffer[offset++] = (id >> 16) & 0xFF;
    buffer[offset++] = (id >> 8) & 0xFF;
    buffer[offset++] = id & 0xFF;

    uint32_t val;
    memcpy(&val, &ax, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    memcpy(&val, &ay, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    memcpy(&val, &az, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;

    buffer[offset++] = trackerId;

    _udp.beginPacket(_serverIp, _serverPort);
    _udp.write(buffer, offset);
    _udp.endPacket();
}

void SlimeUdpClient::sendBattery(float voltage, float batteryPercentage) {
    uint8_t buffer[24];
    int offset = 0;

    int packetType = 12; // BATTERY_LEVEL
    buffer[offset++] = (packetType >> 24) & 0xFF;
    buffer[offset++] = (packetType >> 16) & 0xFF;
    buffer[offset++] = (packetType >> 8) & 0xFF;
    buffer[offset++] = packetType & 0xFF;

    long id = nextPacketId();
    buffer[offset++] = (id >> 56) & 0xFF;
    buffer[offset++] = (id >> 48) & 0xFF;
    buffer[offset++] = (id >> 40) & 0xFF;
    buffer[offset++] = (id >> 32) & 0xFF;
    buffer[offset++] = (id >> 24) & 0xFF;
    buffer[offset++] = (id >> 16) & 0xFF;
    buffer[offset++] = (id >> 8) & 0xFF;
    buffer[offset++] = id & 0xFF;

    uint32_t val;
    memcpy(&val, &voltage, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;
    
    // Percentage needs to be fraction
    float pct = batteryPercentage / 100.0f;
    memcpy(&val, &pct, sizeof(float));
    buffer[offset++] = (val >> 24) & 0xFF; buffer[offset++] = (val >> 16) & 0xFF; buffer[offset++] = (val >> 8) & 0xFF; buffer[offset++] = val & 0xFF;

    _udp.beginPacket(_serverIp, _serverPort);
    _udp.write(buffer, offset);
    _udp.endPacket();
}
