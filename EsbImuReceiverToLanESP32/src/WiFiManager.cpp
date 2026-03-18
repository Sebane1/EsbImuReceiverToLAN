#include "WiFiManager.h"
#include "WiFiConfig.h"
#include "SerialManager.h"
#include "config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

WebServer WiFiManager::server(80);
DNSServer WiFiManager::dnsServer;
bool WiFiManager::portalActive = false;
uint32_t WiFiManager::portalStartTime = 0;

void WiFiManager::init() {
    WiFi.mode(WIFI_STA);
    
    char ssid[33] = {0};
    char pass[65] = {0};
    bool hasSaved = WiFiConfig::hasCredentials();

    if (hasSaved) {
        WiFiConfig::load(ssid, pass);
        DEBUG_PRINT("Connecting to stored WiFi: ");
        DEBUG_PRINTLN(ssid);
        WiFi.begin(ssid, pass);
        WiFi.setSleep(false); // Disable Power Save for low-latency UDP
    } else {
        DEBUG_PRINTLN("No stored credentials. Starting Captive Portal...");
        startCaptivePortal();
        return;
    }

    // Wait for connection (10 seconds timeout)
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        DEBUG_PRINT(".");
        retry++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINTLN("\nConnected!");
        
        // Broadcast discovery info in the exact sequence expected by SlimeVR
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char macLog[40];
        sprintf(macLog, "mac: %02X:%02X:%02X:%02X:%02X:%02X, ", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        SerialManager::logToActivePorts(macLog);
        
        // Signal SlimeVR to start UDP discovery
        SerialManager::logToActivePorts("Searching for the server"); 
        
        DEBUG_PRINT("IP: ");
        DEBUG_PRINTLN(WiFi.localIP());
    } else {
        DEBUG_PRINTLN("\nWiFi Connection Failed.");
        SerialManager::logToActivePorts("Can't connect from any credentials");
        startCaptivePortal();
    }
}

void WiFiManager::startCaptivePortal() {
    portalActive = true;
    portalStartTime = millis();
    
    String apName = WiFiConfig::getDeviceName();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName.c_str());
    
    DEBUG_PRINT("Access Point: ");
    DEBUG_PRINTLN(apName);
    DEBUG_PRINT("AP IP: ");
    DEBUG_PRINTLN(WiFi.softAPIP());

    dnsServer.start(53, "*", WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.onNotFound(handleNotFound);
    server.begin();
}

void WiFiManager::loop() {
    if (portalActive) {
        dnsServer.processNextRequest();
        server.handleClient();
        
        // Auto-reboot after 5 minutes of inactivity in portal? 
        // Or just stay there forever until configured. 
    }
}

bool WiFiManager::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void WiFiManager::reconnect() {
    DEBUG_PRINTLN("Forcing WiFi Reconnection...");
    WiFi.disconnect(false, true); // Just disconnect, don't erase NVS yet (WiFiConfig already saved)
    init(); // Trigger fresh connection attempt and logs
}

void WiFiManager::handleRoot() {
    String html = "<html><head><title>SlimeNRF WiFi Setup</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:sans-serif;background:#1a1a1a;color:#eee;display:flex;flex-direction:column;align-items:center;padding:20px;}";
    html += "input{width:100%;max-width:300px;padding:10px;margin:10px 0;background:#333;color:#fff;border:1px solid #555;border-radius:5px;}";
    html += "button{padding:10px 20px;background:#00aaff;color:#fff;border:none;border-radius:5px;cursor:pointer;}";
    html += "section{background:#2a2a2a;padding:20px;border-radius:10px;box-shadow:0 10px 30px rgba(0,0,0,0.5);}</style></head><body>";
    html += "<h2>SlimeNRF Repeater</h2><section>";
    html += "<form method='POST' action='/save'>";
    html += "SSID:<br><input type='text' name='ssid' placeholder='WiFi Name'><br>";
    html += "Password:<br><input type='password' name='pass' placeholder='Password'><br>";
    html += "<button type='submit'>Save & Connect</button></form></section></body></html>";
    server.send(200, "text/html", html);
}

void WiFiManager::handleSave() {
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    
    if (ssid.length() > 0) {
        DEBUG_PRINT("Saving new WiFi: ");
        DEBUG_PRINTLN(ssid);
        WiFiConfig::save(ssid.c_str(), pass.c_str());
        
        server.send(200, "text/html", "<html><body><h1>Settings Saved!</h1><p>The device is rebooting... connect to your WiFi to continue.</p></body></html>");
        delay(2000);
        ESP.restart();
    } else {
        server.send(200, "text/html", "<html><body><h1>Error</h1><p>SSID cannot be empty.</p><a href='/'>Back</a></body></html>");
    }
}

void WiFiManager::handleNotFound() {
    // Captive portal trick: redirect everything to root
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}
