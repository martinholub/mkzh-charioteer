#include <TheThingsNetwork.h>

const char *appEui = "70B3D57ED0010035";
const char *appKey = "962841F54F3BCFECB8317031276C86D3";

#define loraSerial Serial1
#define debugSerial Serial

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868
    
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
    
void setup() {
  loraSerial.begin(57600);
  debugSerial.begin(9600);
      
  // Initialize LED output pin
  pinMode(LED_BUILTIN, OUTPUT);
    
  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000);
    
  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
}

void loop() {
  debugSerial.println("-- LOOP");
    
  // Prepare array of 1 byte to indicate LED status
  byte data[2];
  data[0] = 1;
  data[1] = 255;
  
  // Send it off
  ttn.sendBytes(data, sizeof(data));
      
  delay(10000);
}
