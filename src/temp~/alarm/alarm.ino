//#include <dht11.h>
#include <NewPing.h>

int echoPin = 11;
int trigPin = 10;
int buzzPin = 12;
int ledGreen = 6;
int ledRed = 4;
int thresh = 3; // <thresh> cm for alarm
int duration, inches, cm; // establish variables for duration of the ping, and the distance result

NewPing sonar(trigPin, echoPin, 200); //sets up the sonar function and limits distance to 200 cm
//dht11 DHT11;
//#define DHT11PIN 7

void setup() {
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(buzzPin, OUTPUT);
  Serial.begin(9600); //sets up serial monitor
}

void loop() {
  delay(100);
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm());
  Serial.println("cm");

  //warning
  if(sonar.ping_cm()<thresh) {
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, LOW);
    digitalWrite(buzzPin, HIGH);
  }

  //no warning
  if(sonar.ping_cm()>=thresh) {
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledRed, LOW);
    digitalWrite(buzzPin, LOW);
  }
}//end loop
