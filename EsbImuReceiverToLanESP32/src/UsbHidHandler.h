#pragma once

#include <Arduino.h>

class SlimeUdpClient;

class UsbHidHandler {
public:
    UsbHidHandler();
    void begin(SlimeUdpClient* udpClient);
    void loop();
    uint32_t getLastReportTime() const { return _lastReportTime; }
    void updateActivity() { _lastReportTime = millis(); }

private:
    SlimeUdpClient* _udpClient;
    uint32_t _lastReportTime;
};
