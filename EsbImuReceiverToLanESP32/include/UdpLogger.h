#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>

class UdpLoggerClass : public Print {
private:
    WiFiUDP udp;
    const char* targetIp;
    uint16_t targetPort;
    bool ready;
    char buffer[256];
    size_t bufferIdx;

public:
    UdpLoggerClass();
    void begin(const char* ip, uint16_t port);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buf, size_t size);
    void flush_buffer();
};

extern UdpLoggerClass UdpLogger;
