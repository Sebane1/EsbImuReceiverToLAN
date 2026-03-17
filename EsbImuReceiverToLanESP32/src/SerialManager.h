#pragma once

#include <Arduino.h>

class SerialManager {
public:
    static void init();
    static void deinit();
    static void loop();
    static void logToActivePorts(String message);
    static void logHeapStatus();


private:
    static void processCommand(String line, Stream& port);
    static void handleSetWifi(String args, Stream& port);
    static void handleSetBWifi(String args, Stream& port);
    static void printStatus(Stream& port);
    
    static String decodeBase64(String input);
};
