# A Simple Arduino Mavlink Library

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/reference/en/libraries/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


I was looking for an easy to implement solution to control my RC vehicles (which are using an ESP32 micro controller) with the help of a Mobile Application. I decided to use [QGroundControl](http://qgroundcontrol.com/) which is based on [Mavlink](https://mavlink.io/en/). 

The project provides the following features:

- It is available as __Arduino Library__ (which should work on any architecture)
- you can easily __override any method__ if you desire a different behavior
- A simple, extendable __parameter store__ is provided, so that you can access the parameter values
- There are examples for __Bluetooth__, __TCP/IP__, and __UDP__ (for the ESP32)
- It is implemented as a header-only C++ library

My code relies heavily on the [Mavlink C library v2](https://github.com/mavlink/c_library_v2) which is released under the MIT license.

## Bluetooth Example
Here is a simple demo that shows how to use it on an ESP32 with Bluetooth:

```C++
#include "Mavlink.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
MavlinkController Drone(&SerialBT);


void setup() {
  // setup log
  Serial.begin(115200);
  SerialBT.begin("Airplane"); // Bluetooth device name

  // provide some initial optional values
  Drone.setValue(GPS_LATITUDE, 46.211236); 
  Drone.setValue(GPS_LONGITUDE, 7.259520);
  Drone.setValue(GPS_ALTITUDE, 503);
  Drone.setValue(VOLTAGE_BATTERY_MV, 5000.0); // Report 5V
}

void loop() {
  Drone.setValue(RECEIVER_RSSI, WiFi.RSSI());
  Drone.loop();  
}

```
This is pretty easy!

## A full RC Airplane
You can easily define a full implementation of an airplane by adding a Servo library. Here is an example for the ESP32 using UDP:

```C++
#include "WiFi.h" 
#include "Mavlink.h"
#include "ESP32Servo.h"

const int port = 14550;
const char* ssid = "Your SSID";
const char* password = "Your Password";
SimpleUDP udp; 
MavlinkController drone(&udp);
Servo servos[4];
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPins[] = {12,13,14,15};

void connectToWifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to "); 
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  Serial.print("Connected with IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  // setup log
  Serial.begin(115200);
  
  // setup Wifi
  connectToWifi();
  
  // start the server
  udp.begin(port);

  // Sion Airport 46.2227° N, 7.3379° E
  drone.setValue(VOLTAGE_BATTERY, 5.0);
  drone.setValue(GPS_LATITUDE, 46.2227);
  drone.setValue(GPS_LONGITUDE, 7.3379);
  drone.setValue(GPS_ALTITUDE, 482.0);
  drone.setValue(HEADING, 350);
  // drone.setLog(Serial);

  // setup servos
  for (int j=0;j<4;j++){
    servos[j].attach(servoPins[j],-100,100);
  }
}

void logControl() {
  static char buffer[1024];
  static long counter;
  // just print every 10000th record
  if (counter++ % 10000 == 0) {
     Serial.print(drone.isArmed()?"[Armed] ":"[Not armed] ");
     Serial.print(drone.getValue(THROTTLE));
     Serial.print(", ");
     Serial.print(drone.getValue(YAW));
     Serial.print(", ");
     Serial.print(drone.getValue(PITCH));
     Serial.print(", ");
     Serial.print(drone.getValue(ROLL));
     Serial.println();
  }
}

void loop() {
  drone.setValue(RECEIVER_RSSI, WiFi.RSSI());
 
  drone.loop();

  // output values to servos
  for (int j=0;j<4;j++){
  if (drone.isArmed() || j > 0) {
      servos[j].write(drone.getValue(j));
    }
  }
    
  logControl();
}

```

## Setting Up QGroundControl

- Define the __Communication Link__: e.g. UDP. The IP address is displayed when you start the sketch
- In General (Settings) in the Fly Vue Section activate the __Visual Joystick__
- If you click on the 'Airplane' icon you can see and use the __joysticks__
- Click on __Arm__ and confirm

and now you are __ready to fly__!

## Installation
You can download this project as a ZIP and in the Arduino IDE use -> Sketch -> Include Library -> Add ZIP Library.

The recommended way, however, is to clone the project to your libraries directory. E.g. with

```
cd ~/Documents/Arduino/libraries
git clone https://github.com/pschatzmann/ArduinoMavlink.git
```