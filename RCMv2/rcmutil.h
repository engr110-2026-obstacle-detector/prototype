#ifndef RCMUTIL_H
#define RCMUTIL_H

// contains functions common to all RCMv2 projects

#include <Arduino.h>

#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0

extern void PowerOn();
extern void Always();
extern void configWifi();
extern void WifiDataToParse();
extern void WifiDataToSend();

void setup()
{
    Serial.begin(115200);

    PowerOn();

    configWifi();
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);

}

boolean connectedToWifi()
{
    return EWD::wifiConnected;
}
boolean connectionTimedOut()
{
    return EWD::timedOut();
}


void loop()
{
    EWD::runWifiCommunication();
    Always();
}

#endif // RCMUTIL_H
