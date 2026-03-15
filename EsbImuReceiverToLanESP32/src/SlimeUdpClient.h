#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>
#include "config.h"

class SlimeUdpClient {
public:
    SlimeUdpClient();
    void begin(const char* ip, uint16_t port);
    void setHardwareAddress(uint8_t mac[6]);
    
    void forceHandshake();
    void loop();

    void sendHandshake();
    void sendHeartbeat();
    void addTracker(uint8_t trackerId, int imuType);
    void sendRotation(uint8_t trackerId, float qx, float qy, float qz, float qw);
    void sendAcceleration(uint8_t trackerId, float ax, float ay, float az);
    void sendBattery(float voltage, float batteryPercentage);

private:
    long nextPacketId();

    WiFiUDP _udp;
    IPAddress _serverIp;
    uint16_t _serverPort;
    uint8_t _hardwareAddress[6];

    long _packetId;
    int _protocolVersion;

    bool _handshakeOngoing;
    bool _isInitialized;
    uint32_t _lastHeartbeatTime;
};
