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
    uint32_t lastPacketReceivedTime = 0;
    uint32_t lastSendDataTime = 0;
    uint32_t lastBatterySendTime = 0;
    uint32_t lastErrMemTime = 0;
    uint16_t handshakeRetryCount = 0;
    int imuType = 0;
    int boardType = 0;
    int mcuType = 0;
    char firmware[32] = {0};
    WiFiUDP udp;
};

class SlimeUdpClient {
public:
    SlimeUdpClient();
    void begin(const char* ip, uint16_t port);
    void onWiFiDisconnect();
    void onWiFiConnect();
    void initializeTracker(uint8_t trackerIndex, const uint8_t mac[6], int imuType, int boardType, int mcuType, const char* firmwareVersion = "Bootleg Tracker ESB");
    void loop();
    bool isHandshakeSuccessful();

    VirtualTracker* getTracker(uint8_t trackerIndex) {
        if(trackerIndex >= 12) return nullptr;
        return &_trackers[trackerIndex];
    }

    bool isNetworkReady(uint8_t trackerIndex);

    void sendRotation(uint8_t trackerIndex, float qx, float qy, float qz, float qw);
    void sendAcceleration(uint8_t trackerIndex, float ax, float ay, float az);
    void sendBattery(uint8_t trackerIndex, float voltage, float batteryPercentage);

private:
    IPAddress _serverIp;
    uint16_t _serverPort;
    bool _discoveryMode = true;
    int _protocolVersion;

    VirtualTracker _trackers[12];

    long nextPacketId(uint8_t trackerIndex);
    void sendHandshake(uint8_t trackerIndex, const char* firmwareVersion);
    void sendHeartbeat(uint8_t trackerIndex);
    void addTracker(uint8_t trackerIndex, int imuType, const char* firmwareVersion);
};
