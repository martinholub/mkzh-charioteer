/* notime main application
 * Checks orientation, temperature, humidity and proximity to other objects. 
 * Alerts user with beep if some values critical.
 * 
 * Done with Arduino UNO, for TTN UNO, double check ports!
 * 
 * ADXL345 / Arduino UNO (! not TTN UNO which is Leonardo) wiring:
 * GND / GND
 * VCC / 5V
 * CS / 5V
 * SDA / A4
 * SCL / A5
 * Note that custom accelerometer is used. I didn't get the provided one to work properly.
 * 
 * DHT-11 / Arduino UNO (! not TTN UNO which is Leonardo) wiring:
 * + / 5V
 * - / GND
 * S / 7
 * 
 * US100 / Arduino UNO (! not TTN UNO which is Leonardo) wiring:
 * VCC / 5V
 * GND / GND
 * Echo / 11
 * Trig / 10
 * 
 * Other wiring:
 * red led: + / 4 with 220R, - / GND
 * green led: + / 8 with 220R, - / GND
 * active buzzer: + / 12, - / GND with 100R
 * 
 * TODO:
 * - Take averages over time.
 * - Implement 3US distance meters to get object dimensions
 * - Use MKZH metal box as a protyping platform
 * 
 * NicetoHaves:
 * - Translate Acceleration input to degrees (either find library or copy-paste math)
 * - Get a power supply for breadboard (6-12V)
 * - Code in other IDE
 * - Refactor, unify naming conventions
 * - Different sounds for different conditions (use tone(), noTone())
*/

#include <Wire.h>  //arduino IDE contains I2C library
#include <dht11.h> // for temp + humidity
#include <stdio.h> // for sprintf
#include <NewPing.h> // for US distance
#include <RunningAverage.h> // For running averages

/*Accelerometer ADXL345 */
#define Register_ID 0    //Address defined ADXL345 register
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37
                    
#define Reg_OFSX 0x1E     //ADXL345 correct register address definitions
#define Reg_OFSY 0x1F
#define Reg_OFSZ 0x20
#define Reg_PWR_CTL 0x2D

int ADXAddress = 0xA7>>1;  //Converted to 7 address
int reading = 0;
int val = 0;
int X0,X1,X_out;
int Y0,Y1,Y_out;
int Z1,Z0,Z_out;
double Xg,Yg,Zg;

/* DHT11 */
dht11 DHT11;
#define DHT11PIN 7

/*US100, ultrasound distance measurement*/
int echoPin = 11;
int trigPin = 10;
int ledGreen = 8;
int ledRed = 4;
int duration, inches, cm; // vars for duration of the ping, and the distance
NewPing sonar(trigPin, echoPin, 100); //init sonar and limit distance 100 cm

/*Sound Alarm*/
int buzzPin = 12;

/*Running Average*/
int ra_length = 10;
RunningAverage myRA_T(ra_length);
int num_samples = 0;

/* Globals */
#define us_per 1 // Define periods with which we take and report measurements
#define accel_per 5
#define temp_per 5
#define global_delay 1000
int max_per = max(max(us_per, accel_per), temp_per);
//int fctr = (int)(temp_per/accel_per); // not used
int i = 0;
int us_thresh = 10; // <thresh> cm for alarm
float grav_thresh = 0.20; //<thresh> Xg|Yg for alarm
int th_thresh[4] = {0, 30, 20, 80}; // <thresh> Tlow, Thigh, HumLow, HumHigh for alarm
bool do_alert = false;

/*********************************************************************/
void setup(){
  Serial.begin(9600);
  delay(100);
  // ADXL345 Setup
  Wire.begin();           //Initializes I2C
  setAccReg(0x31,0x0B);   //Measuring range, plus or minus 16g, 13 bit pattern
  setAccReg(0x2C,0x08);   //Reference rate is set to 12.5 Page pdf13
  setAccReg(0x2D,0x08);   //Select Power Mode Reference pdf24 page
  setAccReg(0x2E,0x80);   //Interrupt Enable DATA_READY
  setAccReg(0x1E,0x00);   //X Offset pdf29 page written test based on the state of the sensor
  setAccReg(0x1F,0x00);   //Y offset pdf29 page written test based on the state of the sensor
  setAccReg(0x20,0x05);   //Z offset pdf29 page written test based on the state of the sensor
  delay(100);
  Wire.beginTransmission(ADXAddress);
  Wire.write(Register_2D);
  Wire.write(8);
  Wire.endTransmission();
  delay(500);
  // Report settings
  Serial.print("Measurement freqs: G, T: ");
  Serial.print(accel_per);Serial.print(", ");Serial.println(temp_per);
  //  Serial.print("\t Factor: ");Serial.println(fctr);

  //US Setup
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(buzzPin, OUTPUT);

  // Rolling Average Setup
  Serial.print("Running Average Version: ");
  Serial.println(RUNNINGAVERAGE_LIB_VERSION);
  myRA_T.clear(); // explicitly start clean
}
/*********************************************************************/
void loop(){
  if(!do_alert){// !do_alert
    digitalWrite(buzzPin, LOW);
    //noTone(buzzPin);
    }
  i = 1;
  while(i<=max_per){
    do_alert = false; // reset warnings
      // Accelerometer ------------------------------------
    if(i%accel_per==0){
      Wire.beginTransmission(ADXAddress);//Read X-axis data
      Wire.write(Register_X0);
      Wire.write(Register_X1);
      Wire.endTransmission();
      Wire.requestFrom(ADXAddress,2);
      if(Wire.available()<=2){
        X0 = Wire.read();
        X1 = Wire.read();
        X1 = X1<<8;
        X_out = X0+X1;
      }
      Wire.beginTransmission(ADXAddress);//Read y-axis data
      Wire.write(Register_Y0);
      Wire.write(Register_Y1);
      Wire.endTransmission();
      Wire.requestFrom(ADXAddress,2);
      if(Wire.available()<=2){
        Y0 = Wire.read();
        Y1 = Wire.read();
        Y1 = Y1<<8;
        Y_out = Y0+Y1;
      }
      Wire.beginTransmission(ADXAddress);//Read z-axis data
      Wire.write(Register_Z0);
      Wire.write(Register_Z1);
      Wire.endTransmission();
      Wire.requestFrom(ADXAddress,2);
      if(Wire.available()<=2){
        Z0 = Wire.read();
        Z1 = Wire.read();
        Z1 = Z1<<8;
        Z_out = Z0+Z1;
      }
      Xg = X_out/256.00;   //The output is converted to gravitational acceleration g, 2 decimal places later.
      Yg = Y_out/256.00;
      Zg = Z_out/256.00;
      // Report values ---------------------------------------------------------------
      Serial.print("i:");Serial.print(i);
      Serial.print(":Grav. Accel.: X="); //So that the display text ' X ='
      Serial.print(Xg);
      Serial.print("\t Y=");
      Serial.print(Yg);
      Serial.print("\t Z=");
      Serial.println(Zg);

      // alarm
      if((abs(Xg)> grav_thresh) || (abs(Yg) > grav_thresh)){
        do_alert = true;
      }
        
    } // end if accel

    if(i%temp_per==0){
      // DHT-11, temperature + humidity ----------------------------------------
      int chk = DHT11.read(DHT11PIN);
      float hm = (float)DHT11.humidity;
      float tm = (float)DHT11.temperature;
      
      Serial.print("Humidity:");
      Serial.print(hm, 2);
      Serial.print(" % ");
      Serial.print("\t Temp:    ");// Print a message of "Temp: "to the LCD.
      Serial.print(tm, 2);// Print a centigrade temperature to the LCD. 
      Serial.println(" C "); // Print the unit of the centigrade temperature to the LCD.
      // alarm
      bool ct = (tm<th_thresh[0])||(tm>th_thresh[1]);
      bool ch = (hm<th_thresh[2])||(hm>th_thresh[3]);
      if (ch || ct){
        do_alert = true;
      }
      // Running Average of Temperature
      myRA_T.addValue(tm); // Add value to buffer on each measurement
      num_samples++;
      if (num_samples>=ra_length){
        Serial.print("Average Temperature for 10 measurements: ");
        Serial.print(myRA_T.getAverage(), ); Serial.println(" C");
        num_samples = 0;
        }
        
    } // end if temp
   
    // US100, ultrasound distance ----------------------------------------
    if(i%us_per==0){
      Serial.print("Ping: ");
      Serial.print(sonar.ping_cm());
      Serial.println("cm");
      
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledRed, LOW);
      //warning
      if(sonar.ping_cm()<us_thresh) {
        digitalWrite(ledRed, HIGH);
        digitalWrite(ledGreen, LOW);
        do_alert = true;
      }
      //no warning
      if(sonar.ping_cm()>=us_thresh) {
        digitalWrite(ledGreen, HIGH);
        digitalWrite(ledRed, LOW);
      }
    } // end if us

    // Buzzer warning -------------------------------------------------------------
    if(do_alert){
      digitalWrite(buzzPin, HIGH);
      //tone(buzzPin, 1000, 500); // freq 1000, duration 500 ms
    } else {
      digitalWrite(buzzPin, LOW);
    }
    // Housekeeping -------------------------------------------------------------
    delay(global_delay);
    i++; // increment
  } // end while
}
/* --- Function Declarations ---*/
/*ADXL345 GY-291 ... accelerometer*/
void setAccReg(int reg,byte value){
  setReg(ADXAddress,reg,value);    //ADXAddress = 0xA7>>1
}
void setReg(int device,int reg,byte value){
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
/* DHT-11 ... temp + humidity*/
