#include "UdpLogger.h"

UdpLoggerClass UdpLogger;

UdpLoggerClass::UdpLoggerClass() : ready(false), bufferIdx(0) {}

void UdpLoggerClass::begin(const char* ip, uint16_t port) {
    targetIp = ip;
    targetPort = port;
    udp.begin(port + 1);
    ready = true;
}

size_t UdpLoggerClass::write(uint8_t c) {
    // Always fall back to serial
    Serial.write(c);

    if (!ready) return 1;
    
    buffer[bufferIdx++] = c;
    if (c == '\n' || bufferIdx >= sizeof(buffer) - 1) {
        flush_buffer();
    }
    return 1;
}

size_t UdpLoggerClass::write(const uint8_t *buf, size_t size) {
    // Always fall back to serial
    Serial.write(buf, size);

    if (!ready) return size;
    
    // Flush pending chars
    if (bufferIdx > 0) flush_buffer();
    
    udp.beginPacket(targetIp, targetPort);
    udp.write(buf, size);
    udp.endPacket();
    return size;
}

void UdpLoggerClass::flush_buffer() {
    if (bufferIdx > 0 && ready) {
        if (udp.beginPacket(targetIp, targetPort)) {
            udp.write((const uint8_t*)buffer, bufferIdx);
            udp.endPacket();
        }
        bufferIdx = 0;
    }
}
