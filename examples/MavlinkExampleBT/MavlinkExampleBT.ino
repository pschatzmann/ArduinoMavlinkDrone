

/**
 *  This is a small demo which is based on the BluetoothSerial. 
 */
#include "SimpleMavlinkDrone.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
SimpleMavlinkDrone Drone(&SerialBT);


void setup() {
  // setup log
  Serial.begin(115200);
  SerialBT.begin("Airplane"); //Bluetooth device name
}

void loop() {
  Drone.setValue(RSSI, WiFi.RSSI()); 
  Drone.loop();  

}
