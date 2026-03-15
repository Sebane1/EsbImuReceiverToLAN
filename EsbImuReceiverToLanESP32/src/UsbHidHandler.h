#pragma once

#include <Arduino.h>

class SlimeUdpClient;

class UsbHidHandler {
public:
    UsbHidHandler();
    void begin(SlimeUdpClient* udpClient);
    void loop();

private:
    SlimeUdpClient* _udpClient;
};
