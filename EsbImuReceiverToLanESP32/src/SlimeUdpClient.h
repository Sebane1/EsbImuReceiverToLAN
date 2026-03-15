#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>

struct VirtualTracker {
    bool active = false;
    uint8_t hardwareAddress[6] = {0};
    long packetId = 0;
    bool handshakeOngoing = false;
    bool isInitialized = false;
    uint32_t lastHeartbeatTime = 0;
    uint32_t lastHandshakeTime = 0;
    int imuType = 0;
    WiFiUDP udp; // Each tracker needs its own distinct UDP socket (local port)
};

class SlimeUdpClient {
public:
    SlimeUdpClient();
    void begin(const char* ip, uint16_t port);
    void initializeTracker(uint8_t trackerIndex, const uint8_t mac[6], int imuType);
    void loop();

    void sendRotation(uint8_t trackerIndex, float qx, float qy, float qz, float qw);
    void sendAcceleration(uint8_t trackerIndex, float ax, float ay, float az);
    void sendBattery(uint8_t trackerIndex, float voltage, float batteryPercentage);

private:
    IPAddress _serverIp;
    uint16_t _serverPort;
    int _protocolVersion;

    VirtualTracker _trackers[10];

    long nextPacketId(uint8_t trackerIndex);
    void sendHandshake(uint8_t trackerIndex);
    void sendHeartbeat(uint8_t trackerIndex);
    void addTracker(uint8_t trackerIndex, int imuType);
};
