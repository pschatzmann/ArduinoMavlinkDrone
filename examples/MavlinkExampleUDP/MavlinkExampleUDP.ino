
/**
 * This is a small Demo which shows how we can use UDP for the communication. We can use our SimpleUDP class which is 
 * an simple extension of the Arduno WiFiUDP class. 
 */
#include "WiFi.h" 
#include "SimpleUDP.h"
#include "SimpleMavlinkDrone.h"

const int port = 14550;
const char* ssid = "Phil Schatzmann";
const char* password = "sabrina01";
SimpleUDP udp; //WiFiUDP udp;
SimpleMavlinkDrone drone(&udp);


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
  
  // start the  server
  udp.begin(port);
}

void loop() {
    drone.loop();  
}