#include "WiFiConfig.h"
#include <WiFi.h>

static const char* PREFS_NAMESPACE = "wifi-config";
static const char* KEY_SSID = "ssid";
static const char* KEY_PASS = "pass";

void WiFiConfig::load(char* ssid, char* password) {
    Preferences prefs;
    prefs.begin(PREFS_NAMESPACE, true);
    
    String s = prefs.getString(KEY_SSID, "");
    String p = prefs.getString(KEY_PASS, "");
    
    if (ssid) strcpy(ssid, s.c_str());
    if (password) strcpy(password, p.c_str());
    
    prefs.end();
}

void WiFiConfig::save(const char* ssid, const char* password) {
    Preferences prefs;
    prefs.begin(PREFS_NAMESPACE, false);
    
    prefs.putString(KEY_SSID, ssid);
    prefs.putString(KEY_PASS, password ? password : "");
    
    prefs.end();
}

bool WiFiConfig::hasCredentials() {
    Preferences prefs;
    prefs.begin(PREFS_NAMESPACE, true);
    bool exists = prefs.isKey(KEY_SSID);
    prefs.end();
    return exists;
}

void WiFiConfig::clear() {
    Preferences prefs;
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.clear();
    prefs.end();
}

String WiFiConfig::getDeviceName() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char name[32];
    sprintf(name, "SlimeNRFRepeater-%02X%02X", mac[4], mac[5]);
    return String(name);
}
