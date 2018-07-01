#include <TheThingsNetwork.h>

const char *appEui = "70B3D57ED001002B";
const char *appKey = "8A00E8AAF118BCE0768225E14A5419CE";

#define loraSerial Serial1
#define debugSerial Serial

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868
    
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
    
void setup() {
  loraSerial.begin(57600);
  debugSerial.begin(9600);
      
  // Initialize LED output pin
  // pinMode(LED_BUILTIN, OUTPUT);
    
  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000);
    
  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
}

void loop() {
  // pass
}
