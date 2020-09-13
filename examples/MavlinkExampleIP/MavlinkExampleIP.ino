/**
 *  In this demo we use TCP/IP to communicate. This is a little bit more difficult then the other demos because we need to 
 *  setup up a server that is managing the connections first before we get the a Client - which is the Stream that we need to
 *  use in our SimpleMavlinkDrone.
 */

#include "WiFi.h" // ESP32 WiFi include
#include "Ethernet.h"
#include "SimpleMavlinkDrone.h"

const int port = 5760;
const char* ssid = "Phil Schatzmann";
const char* password = "sabrina01";
WiFiServer server(port);
SimpleMavlinkDrone drone;

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
  
  // start the tcp ip server
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client){
    // we did not define the Stream in the constructor so we assign it here
    drone.reconnect(&client);
    while(client.connected()){
        drone.loop();  
        delay(10);
    }
  }
}