/*
HC-SR04 Ping distance sensor]
VCC to arduino 5v GND to arduino GND
Echo to Arduino pin 12 Trig to Arduino pin 13
Red POS to Arduino pin 11
Green POS to Arduino pin 10
560 ohm resistor to both LED NEG and GRD power rail
*/

const int trigPin = 13;
const int echoPin = 11;
const int red = 10;
const int  blue = 9;

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
}

void loop() {
  float duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  //delayMicroseconds(1000); //- Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  
  if (distance < 4) {  // This is where the LED On/Off happens
    digitalWrite(red, HIGH); // When the Red condition is met, the Green LED should turn off
    digitalWrite(blue, LOW);
  } else {
    digitalWrite(red, LOW);
    digitalWrite(blue, HIGH);
  }
  if (distance >= 200 || distance <= 0){
    Serial.println("Out of range");
    Serial.println(duration);
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }
  delay(1000);
}
