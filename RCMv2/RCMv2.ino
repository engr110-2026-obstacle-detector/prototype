//   prototype obstacle detector
//   based on the https://github.com/rcmgames/RCMv2 template

#include <Arduino.h>

#include <ESP32_easy_wifi_data.h>
#include <VL53L1X.h>
#include <Wire.h>

// https://registry.platformio.org/libraries/pololu/VL53L1X/examples/ContinuousMultipleSensors/ContinuousMultipleSensors.ino
//  The number of sensors in your system.
const uint8_t sensorCount = 3;

#define LEFT 0
#define FRONT 1
#define RIGHT 2

float distances[sensorCount];

float objectThreshold=100;
float dropThreshold=300;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = { A0, A1, A2 };

VL53L1X sensors[sensorCount];

boolean objectFlags[sensorCount] = { false, false, false };
boolean dropFlags[sensorCount] = { false, false, false };
boolean prevObjectFlags[sensorCount] = { false, false, false };
boolean prevDropFlags[sensorCount] = { false, false, false };

byte objectCounts[sensorCount] = { 0, 0, 0 };
byte dropCounts[sensorCount] = { 0, 0, 0 };

uint8_t eventDetector(unsigned long currentMillis, uint8_t sensorIndex, float distance){
    // simple object/drop event detector based on distance threshold
    // returns 1 for object detected, -1 for drop detected, 0 for nothing
    if(distance<objectThreshold){
        return 1;
    } else if (distance>dropThreshold){
        return -1;
    }
    return 0;
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here

    Wire1.begin();
    Wire1.setClock(400000); // use 400 kHz I2C

    // Disable/reset all sensors by driving their XSHUT pins low.
    for (uint8_t i = 0; i < sensorCount; i++) {
        pinMode(xshutPins[i], OUTPUT);
        digitalWrite(xshutPins[i], LOW);
    }

    // Enable, initialize, and start each sensor, one by one.
    for (uint8_t i = 0; i < sensorCount; i++) {

        sensors[i].setBus(&Wire1);

        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        pinMode(xshutPins[i], INPUT);
        delay(10);

        sensors[i].setTimeout(500);
        if (!sensors[i].init()) {
            Serial.print("Failed to detect and initialize sensor ");
            Serial.println(i);
            while (1)
                ;
        }

        // Each sensor must have its address changed to a unique value other than
        // the default of 0x29 (except for the last one, which could be left at
        // the default). To make it simple, we'll just count up from 0x2A.
        sensors[i].setAddress(0x2A + i);

        sensors[i].setDistanceMode(VL53L1X::Short);

        sensors[i].setROISize(4, 4);

        sensors[i].startContinuous(50);
    }
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you don't also need to call leftMotorController.run())
    for (uint8_t i = 0; i < sensorCount; i++) {
        // Serial.print(sensors[i].read());
        distances[i] = sensors[i].read();
        Serial.print(distances[i]);
        if (sensors[i].timeoutOccurred()) {
            Serial.print(" TIMEOUT"); // sensor disconnected
        }
        Serial.print('\t');

        prevObjectFlags[i] = objectFlags[i];
        prevDropFlags[i] = dropFlags[i];
        dropFlags[i] = false;
        objectFlags[i] = false;
        int8_t ret=eventDetector(millis(), i, distances[i]); // returns 1 for object, -1 for drop, 0 for nothing
        if (ret==-1){
            dropFlags[i]=true;
        } else if (ret==1){
            objectFlags[i]=true;
        }
        // count how many new objects/drops detected (if number increments that means an alert needs to be given)
        if (objectFlags[i] && !prevObjectFlags[i]) {
            objectCounts[i]++;
        }
        if (dropFlags[i] && !prevDropFlags[i]) {
            dropCounts[i]++;
        }
    }
    Serial.println();

    delay(1);
}

void WifiDataToParse()
{
    EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    objectThreshold = EWD::recvFl();
    dropThreshold = EWD::recvFl();
}
void WifiDataToSend()
{
    EWD::sendFl(millis());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
    for (uint8_t i = 0; i < sensorCount; i++) {
        EWD::sendFl(distances[i]);
    }
    for (uint8_t i = 0; i < sensorCount; i++) {
        EWD::sendBy(objectCounts[i]);
        EWD::sendBy(dropCounts[i]);
    }
}

void configWifi()
{
    // EWD::mode = EWD::Mode::connectToNetwork;
    // EWD::routerName = "router";
    // EWD::routerPassword = "password";
    // EWD::routerPort = 25210;

    EWD::mode = EWD::Mode::createAP;
    EWD::APName = "rcm0";
    EWD::APPassword = "rcmPassword";
    EWD::APPort = 25210;
}

#include "rcmutil.h"
