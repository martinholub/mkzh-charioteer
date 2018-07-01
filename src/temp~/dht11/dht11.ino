/*
*/
 
#include <dht11.h>
 
dht11 DHT11;
#define DHT11PIN 7
 
void setup()
{ 
  // pass
  Serial.begin(9600); // Starts the serial communication
}
 
void loop()
{
    int chk = DHT11.read(DHT11PIN);
    Serial.print("Humidity:");
    Serial.print((float)DHT11.humidity, 2);
    Serial.println(" % ");
    
    Serial.print("Temp:    ");// Print a message of "Temp: "to the LCD.
    Serial.print((float)DHT11.temperature, 2);// Print a centigrade temperature to the LCD. 
    Serial.println(" C "); // Print the unit of the centigrade temperature to the LCD.
    delay(60000);     
}
