/* notime main application
 * Checks orientation, temperature, humidity and proximity to other objects.
 * Alerts user with beep if some values critical.
 *
 * Done with Arduino UNO, for TTN UNO, double check ports!
 *
 * ADXL345 / Arduino UNO / TTN Leonardo
 * GND / GND / GND
 * VCC / 5V / 5V
 * CS / 5V / 5V
 * SDA / A4 / 2
 * SCL / A5 / 3
 * Note that custom accelerometer is used. I didn't get the provided one to work properly.
 *
 * DHT-11 / Arduino UNO + TTN Leonardo
 * + / 5V
 * - / GND
 * S / 7
 *
 * US100 / Arduino UNO + TTN Leonardo
 * VCC / 5V
 * GND / GND
 * Echo / 11
 * Trig / 10
 *
 * u-blox PAM-7Q + LevelShifter
 * **USE ONLY WITH LEVEL CONVERTER, 3.3V device**
 * PAM-7Q → LevelConv → TTN UNO (==TTN Leonardo)
 *                   HV  →   5V   on TTN UNO
 * VCC    →  LV  →          3.3V on TTN UNO
 * TX     →  LVx → HVx  →   TTN UNO 5
 * RX     →  LVx → HVx  →   TTN UNO 13
 * V_BCKP →  nc
 * TIMEPULSE→  nc
 * SDA    →  nc
 * SCA    →  nc
 * S0 & S1 must be bridged to the right (GND)
 *
 * Other wiring:
 * red led: + / 4 with 220R, - / GND
 * green led: + / 8 with 220R, - / GND
 * active buzzer: + / 12, - / GND with 100R
 * themistor (polarity does not matter): a / 5V, b / GND with 10kR and A3
 *
 * TODO:
 * - Take averages over time also for angle
 * - Send messages as payloads
 * - Calculate speed from GPS?
 *   - probably should do in arduino not to have to transmit data
 *
 * NicetoHaves:
 * - Translate Acceleration input to degrees (either find library or copy-paste math)
 * - Get a power supply for breadboard (6-12V)
 * - Code in other IDE
 * - Refactor, unify naming conventions
 * - Different sounds for different conditions (use tone(), noTone())
****************************************************************************/

/* ------------------- Accelerometer ADXL345 ------------------------------*/
#include <Wire.h>  //arduino IDE contains I2C library
#include <Arduino.h> // need to use Arduino files as cpp files
#include <stdio.h> // for printf

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
bool use_accel = true;
float tilt_crit[2] = {0};
uint16_t XgB, YgB;

/* -------------------  DHT11 ------------------------------------*/
#include <dht11.h> // for temp + humidity
#include <RunningAverage.h> // For running averages

dht11 DHT11;
#define DHT11PIN 7

RunningAverage myRA_T(10);
RunningAverage myRA_h(10);

/* -------------------  US100 ------------------------------------*/
///*US100, ultrasound distance measurement*/
//#include <NewPing.h> // for US distance
//int echoPin = 11;
//int trigPin = 10;
//int duration, inches, cm; // vars for duration of the ping, and the distance
//NewPing sonar(trigPin, echoPin, 100); //init sonar and limit distance 100 cm

/* -------------------  Sound (+Light) Alarm ----------------------------*/
int buzzPin = 12;
int ledGreen = 8;
int ledRed = 4;

/* -------------------  GPS u-blox PAM-7Q -------------------------------*/
#include <TinyGPS++.h>
#include <AltSoftSerial.h>

TinyGPSPlus gps;
static const int RXPin = 13, TXPin = 5;
AltSoftSerial gpsSerial(RXPin, TXPin);
// init
int gps_avg_per = 3; // average over <n> measurements, report only then
int gps_counter = 0; // count measurmements with gps

RunningAverage hdops(gps_avg_per); // signal quality
RunningAverage speeds(gps_avg_per); // velocities (as calculated by gps)

uint16_t velB, distB; // velocity and distance as bytes

/* --------------------- Thermistor ---------------------------------*/
int thermistorPin = 3;
int R_therm = 10000;
float logR2, R2, tm_out0, tm_out;
// float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

RunningAverage myRA_Tout(10);
/* ----------------------- SHT - Sensirion ------------------------- */
// #include <SHTSensor.h>

// SHTSensor sht;
// bool use_accel = false;

/* --------------------------- TTN ----------------------------------*/
#include <TheThingsNetwork.h>
#include <private_keys.h>

#define loraSerial Serial1
// #define debugSerial Serial // is needed?

// Use european freq plan
#define freqPlan TTN_FP_EU868
TheThingsNetwork ttn(loraSerial, Serial, freqPlan);
bool is_connected = false;

// construct data structure to hold our messages
// not used because TheThingsMessage does not have countrepart in js
// devicedata_t data = api_DeviceData_init_default;
uint8_t dataBuffer[16] = {0}; // uint8_t is byte
uint32_t latB, lonB; // latitude and lognitude in bytes
uint16_t tInB, tOutB; // temp in and out in bytes
#define send_per 30 //gps_per*gps_avg_per
// utility to get array size
#define COUNT_OF(array) (sizeof(array) / sizeof(array[0]))

/* -------------------  Globals ------------------------------------*/
#define us_per 1 // Define periods with which report measurements
#define accel_per 10 // how often reported
#define temp_per 10 // how often reported
#define gps_per 10 // How often is the data sampled
#define global_delay 1000
int max_per = max(max(max(gps_per, accel_per), temp_per), send_per);
int i = 0;
int num_meas = 1;
int us_thresh = 10; // <thresh> cm for alarm
float grav_thresh = 0.20; //<thresh> Xg|Yg for alarm
int th_thresh[4] = {0, 35, 20, 80}; // <thresh> Tlow, Thigh, HumLow, HumHigh for alarm
bool do_alert = false;

/* --------------- Function Declarations ------------------------------ */
/*ADXL345 GY-291 ... accelerometer*/
void setReg(int device,int reg,byte value){
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setAccReg(int reg,byte value){
  setReg(ADXAddress,reg,value);    //ADXAddress = 0xA7>>1
}

void add_buffer_temp(double tIn, double tOut, int it = 0) {
  // it ... pos where to start writing to the buffer

  tInB = ((tIn + 50) / 100.00) * pow(256,2);
  tOutB = ((tOut + 50) / 100.0) * pow(256,2);

  dataBuffer[it] = ( tInB >> 8 ) & 0xFF;
  dataBuffer[it+1] = tInB & 0xFF;

  dataBuffer[it+2] = ( tOutB >> 8 ) & 0xFF;
  dataBuffer[it+3] = tOutB & 0xFF;
}

void add_buffer_gps(double lat, double lon){

  latB = ((lat + 90) / 180.0) * pow(256,3);
  lonB = ((lon + 180) / 360.0) * pow(256,3);

  dataBuffer[0] = ( latB >> 16 ) & 0xFF;
  dataBuffer[1] = ( latB >> 8 ) & 0xFF;
  dataBuffer[2] = latB & 0xFF;

  dataBuffer[3] = ( lonB >> 16 ) & 0xFF;
  dataBuffer[4] = ( lonB >> 8 ) & 0xFF;
  dataBuffer[5] = lonB & 0xFF;
}

void add_buffer_dyn(float speed, float dist, int it = 0) {
  // Velocity As Bytes

  // higher precision
  // round to decimals, scale and lift to given byte precision
  velB = speed / 99.0 * pow(256,2);
  dataBuffer[it] = (velB >> 8) & 0xFF;
  dataBuffer[it+1] = (velB) & 0xFF;

  // lower precision
  // velB = (round(speed * 100.0) / 100.0);
  // dataBuffer[it] = velB;

  // Distance as Bytes
  // round to decimals, scale and lift to given byte precision
  distB = dist / 9999.0 * pow(256,2);
  dataBuffer[it+2] = (distB >> 8) & 0xFF;
  dataBuffer[it+3] = (distB) & 0xFF;
}

void add_buffer_tilt(float Xg_crit, float Yg_crit, int it = 0) {
  XgB = (Xg_crit  + 2.0) * pow(256,1) / 4;
  YgB = (Yg_crit + 2.0) * pow(256,1) / 4;

  dataBuffer[it] = XgB;
  dataBuffer[it+1] = YgB;
}

void update_accelerometer(){

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
    // alarm
    if((abs(Xg)> grav_thresh) || (abs(Yg) > grav_thresh)){
      do_alert = true;
    }

    // Save Critical tilts
    if (abs(Xg) > abs(tilt_crit[0])) {
      tilt_crit[0] = Xg;
    }
    if (abs(Yg) > abs(tilt_crit[1])) {
      tilt_crit[1] = Yg;
    }

    add_buffer_tilt(tilt_crit[0], tilt_crit[1], 10);

    if(i%accel_per==0){
      Serial.print("i:");Serial.print(num_meas);
      Serial.print(":G.: X="); //So that the display text ' X ='
      Serial.print(Xg);
      Serial.print("\t Y=");
      Serial.print(Yg);
      Serial.print("\t Z=");
      Serial.println(Zg);

      Serial.print("Tilts :");
      Serial.print(tilt_crit[0]); Serial.print(", ");
      Serial.println(tilt_crit[1]);
    } // end if accel

}

void update_temphum() {
  // DHT-11
  DHT11.read(DHT11PIN);
  float hm = (float)DHT11.humidity;
  float tm = (float)DHT11.temperature + 4*(!use_accel); // hardcode correction

  // Thermistor - Ambient Temperature
  float tPin_val = analogRead(thermistorPin);
  //the formula to calculate  temperature in degC
  // R2 = R_therm * (1023.0 / tPin_val - 1.0);
  // logR2 = log(R2);
  // tm_out0 = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  // tm_out = tm_out0 - 273.15;
  int corr = -8; // additional connection
  int B_const = 3435; float RT_const = 25.0; // tune me
  // R2 = (1023.0*R_therm)/tPin_val-R_therm;
  // tm_out0 = B_const/(log(R2/R_therm) + B_const/(273.15+RT_const));
  // tm_out = tm_out0 - 273.15 + corr;
  float R2 = (1023.0*R_therm)/tPin_val-R_therm;
  float tm_out0 = (B_const/(log(R2/R_therm)+(B_const/(273.15+RT_const))));
  tm_out = tm_out0 - 273.15 + corr;

  // SHT -  Ambient Temperature --- not used
  // sht.readSample();
  // float tm_out = sht.getTemperature();

  // alarm
  bool ct = (tm<th_thresh[0])||(tm>th_thresh[1]);
  bool ch = (hm<th_thresh[2])||(hm>th_thresh[3]);
  if (ch || ct){
    do_alert = true;
  }
    // Running Average of Temperature
  myRA_T.addValue(tm); // Add value to buffer on each measurement
  myRA_h.addValue(hm);
  myRA_Tout.addValue(tm_out);
  // Report Values --------------------------------------------------------------
  if(i%temp_per==0){
    if (myRA_T.getCount()>=myRA_T.getSize()){ // T+h measured together, cond. on one
      // Debug
      // Serial.print("Count:");Serial.print(myRA_T.getCount());
      // Serial.print("\t Size:");Serial.println(myRA_T.getSize());

      Serial.print("RAs:\t");
      Serial.print("Hum.:");
      Serial.print(myRA_h.getAverage(), 2);
      Serial.print(" % ");
      Serial.print("\t Temp. Box: ");
      Serial.print(myRA_T.getAverage(), 2);// Print a centigrade temperature
      Serial.print(" C "); // Print the unit of the centigrade temperature
      Serial.print("\t Temp. Amb.: ");
      Serial.print(myRA_Tout.getAverage(), 2);// Print a Cdeg temperature
      Serial.println(" C "); // Print the unit
      // Add data to buffer
      add_buffer_temp(myRA_T.getAverage(), myRA_Tout.getAverage(), 6);
    }
  } // end if temp
}

int update_gps(int j) {
  // Measure only every gps_per passes through loop

  // init
  double loc[gps_avg_per][3]; // location: lat,lng,alt
  int tim[gps_avg_per][4]; // time: hr,mn,sec,csec
  unsigned long time_[gps_avg_per]; // time: millis()
  int n_sat[gps_avg_per]; // number of satellites
  // float hdops[gps_avg_per];
  // double speeds[gps_avg_per];
  // bool l_valid = false, s_valid = false, t_valid = false, h_valid = false;
  if (j==0) {
    RunningAverage hdops(gps_avg_per); // signal quality
    RunningAverage speeds(gps_avg_per); // velocities (as calculated by gps)
    speeds.clear(); hdops.clear(); // explicitly start clean on each new avgg
  }

  if (gps.location.isValid()) {
    loc[j][0] = gps.location.lat();
    loc[j][1] = gps.location.lng();
  } else {
    loc[j][0] = 00.00; loc[j][1] = 00.00;
  }

  if (gps.altitude.isValid()){
    loc[j][2] = gps.altitude.meters();
  } else {
    loc[j][2] = 00.00;
  }

  if (gps.speed.isValid()) { // else try velocity
    speeds.addValue(gps.speed.kmph()/3.6); // speed in m/s
  } else {
    speeds.addValue(99.00);
  }

  time_[j] = millis(); // get time of measurmement
  if (gps.time.isValid()) {
    tim[j][0] = gps.time.hour() + 2; // adjust for time difference
    tim[j][1] = gps.time.minute();
    tim[j][2] = gps.time.second();
    tim[j][3] = gps.time.centisecond();
  } else {
    tim[j][0] = 99; tim[j][1] = 99; tim[j][2] = 99; tim[j][3] = 99;
  }

  if (gps.hdop.isValid()){
    hdops.addValue(gps.hdop.hdop());
  } else {
    hdops.addValue(00.0);
  }

  if (gps.satellites.isValid()) {
    n_sat[j] = gps.satellites.value();
  } else {
    n_sat[j] = 99;
  }

  // Check if data is incoming
  if (millis() > 10000 && gps.charsProcessed() < 10) {
    Serial.print("Connection problem?");
  }

  // Report Values ----------------------------------------------
  // Reporting ----------------------------------------------
  // Time
  // or use <Time> library and get time from board.
  Serial.print("Time: "); // F() stores string in flash memory
  if (tim[j][0] < 10) Serial.print("0");
  Serial.print(tim[j][0]);
  Serial.print(":");
  if (tim[j][1] < 10) Serial.print("0");
  Serial.print(tim[j][1]);
  Serial.print(":");
  if (tim[j][2] < 10) Serial.print("0");
  Serial.print(tim[j][2]);
  Serial.print(".");
  if (tim[j][3] < 10) Serial.print("0");
  Serial.print(tim[j][3]);

  // Location
  Serial.print("\t Location: ");
  Serial.print(loc[j][0], 4);
  Serial.print(", ");
  Serial.print(loc[j][1], 4);
  Serial.print(" alt: ");
  Serial.print(loc[j][2], 4);

  // Signal quality,  HDOP
  Serial.print("\t HDOP: "); Serial.print(hdops.getAverage(), 2);
  Serial.print(", # Satell.: "); Serial.println(n_sat[j]);

  // Add Stuff to Buffer:
  add_buffer_gps(loc[j][0], loc[j][1]);

  if (j >= gps_avg_per-1){
    // Calculations ----------------------------------------------
    if (j >=1) { // we have at least two datapoints
      float tot_dist = 0.0;
      float speeds_[gps_avg_per-1];
      float speed_sum = 0.0;

      for (int k=1; k<=gps_avg_per-1; k++){
        // distance between two following measurmements
        float dist_ = gps.distanceBetween(loc[k-1][0], loc[k-1][1], loc[k][0], loc[k][1]);
        // time delta in seconds between two measurmenets
        // float dt_ = (tim[k-1][0] - tim[k][0])*60*60 + (tim[k-1][1] - tim[k][1])*60 +
        //             (tim[k-1][2] - tim[k][2]) + (tim[k-1][2] - tim[k][2])/100;
        float dt_ = (time_[k] - time_[k-1]) * 1000;

        speeds_[k-1] = dist_ / dt_;
        speed_sum += dist_ / dt_;
        tot_dist += dist_;
      }
      // Distance between corresponding to interval endpoints
      float dist = gps.distanceBetween(loc[0][0], loc[0][1], loc[j][0], loc[j][1]);
      // float sec_delta = (tim[j][0] - tim[k][0])*60*60 + (tim[j][1] - tim[0][1])*60 +
      //               (tim[j][2] - tim[0][2]) + (tim[j][2] - tim[0][2])/100;
      // Duration of whole avg interval
      float sec_delta = (time_[j] - time_[0])*1000;
      int speeds_size = COUNT_OF(speeds_);
      float avg_speed = speed_sum/speeds_size;
      float avg_speed2 = dist / sec_delta;

      // Sanity checks:
      if (avg_speed > 99.0|| avg_speed < 0) {
        avg_speed = 99.0;
      }
      if (avg_speed2 > 99.0 || avg_speed2 < 0) {
        avg_speed2 = 99.0;
      }
      if (dist > 9999.0 || dist < 0) {
        dist = 9999.0;
      }
      if (tot_dist > 9999.0 || tot_dist < 0) {
        tot_dist = 9999.0;
      }
      // Reporting -------------------------------------------------------
      // Speed
      Serial.print("Avg Speed:: (GPS): "); Serial.print(speeds.getAverage(), 2);
      Serial.print(", (deltas): ");Serial.print(avg_speed, 2);
      Serial.print(", (endpoints): ");Serial.println(avg_speed2, 2);

      // Distance
      Serial.print("Dist (endpts.): "); Serial.print(dist, 2);
      Serial.print(", Dist (ints.): ");Serial.print(tot_dist, 2);
      Serial.print(", No. Points: "); Serial.println(speeds_size + 1);

      add_buffer_dyn(avg_speed2, dist, 12);
    }

    j = 0; // Housekeeping
  }
  else {
    j++; // Housekeeping
  }
  return j;
}

void delay_smart(unsigned long ms) {
  // This custom version of delay() ensures that the gps object is "fed"
  unsigned long start = millis(); // Get current time from board start in ms
  do {
    // wait for data available
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    // wait extra if needed to fill delay
  } while (millis() - start < ms);
}

void send_ttn_message(uint8_t* data, size_t data_size, port_t prt = 1) {
    // int data_size = (int)(sizeof(data)/sizeof(data[0]));
    if(is_connected){
      ttn_response_t ret_val = ttn.sendBytes(data, data_size);// sizeof(data)
      Serial.print(":Send Stat.:"); Serial.println(ret_val);
    } else {
      Serial.print("ttn message: ");
      uint8_t i = 0;
      for (i = 0; i < data_size; i++){
        if (data[i] < 16) {
          Serial.print("0");
          Serial.print(data[i], HEX);
        } else {
          Serial.print(data[i], HEX);
        }
      }
      Serial.println("");
    }
}
//void update_us() {
//
//    if(i%us_per==0){
//      Serial.print("Ping: ");
//      Serial.print(sonar.ping_cm());
//      Serial.println("cm");
//
//      digitalWrite(ledGreen, HIGH);
//      digitalWrite(ledRed, LOW);
//      //warning
//      if(sonar.ping_cm()<us_thresh) {
//        digitalWrite(ledRed, HIGH);
//        digitalWrite(ledGreen, LOW);
//        do_alert = true;
//      }
//      //no warning
//      if(sonar.ping_cm()>=us_thresh) {
//        digitalWrite(ledGreen, HIGH);
//        digitalWrite(ledRed, LOW);
//      }
//    } // end if us
//}

/************************************ SETUP *****************************/
void setup(){
  Serial.begin(9600);
  loraSerial.begin(57600);
  gpsSerial.begin(9600); // could uuse loraSerial here?

  while (!Serial && millis() < 1000); // wait till serial ready and 1000 ms

  // u-blox gps
  speeds.clear(); hdops.clear(); // explicitly start clean on each new avg run

  delay(100);
  // ADXL345 Setup
  if (use_accel){
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
  } else {
    // SHT Sensirion setup / cannot be used w/ Accelerometer!
    // Wire.begin();
    // sht.init();
  }

  // Report settings
  Serial.print("Freqs: acc, T, gps, gps_avg: ");
  Serial.print(accel_per);Serial.print(", ");Serial.print(temp_per);
  Serial.print(", "); Serial.print(gps_per);
  Serial.print(", "); Serial.println(gps_avg_per);
  Serial.print("Send freq: "); Serial.println(send_per);

  //Alarm Setup
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(buzzPin, OUTPUT);

  // Rolling Average Setup
  Serial.print("RA ver: ");
  Serial.println(RUNNINGAVERAGE_LIB_VERSION);
  myRA_T.clear(); // explicitly start clean
  myRA_h.clear();
  myRA_Tout.clear();

  // Join the TTN
  Serial.print("--- TTN STATUS ---");
  ttn.showStatus();
  ttn.provision(appEui, appKey);
  is_connected = ttn.join(); // by default infinity tries
  if (!is_connected){
    Serial.print("Join to TTN failed.");
  }
}
/***************************** MAIN ***************************************/
void loop(){

  if(!do_alert){// !do_alert
    digitalWrite(buzzPin, LOW);
    //noTone(buzzPin);
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledRed, LOW);
    }

  i = 1; // while counter
  while(i<=max_per){
    do_alert = false; // reset warnings

    // Accelerometer ------------------------------------
    update_accelerometer();

    // DHT-11, temperature + humidity ----------------------------------------
    update_temphum();

    // US100, ultrasound distance ----------------------------------------------
    //update_us();

    // GPS, u-blox -----------------------------------------------------------
    if (i%gps_per == 0){
      gps_counter = update_gps(gps_counter);
    } // else leave unchaged
    // Buzzer warning -------------------------------------------------------------
    if(do_alert){
      digitalWrite(buzzPin, HIGH);
      //tone(buzzPin, 1000, 500); // freq 1000, duration 500 ms
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledRed, HIGH);
    } else {
      digitalWrite(buzzPin, LOW);
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledRed, LOW);
    }

    // Messaging the TTN Application
    if (num_meas % send_per == 0){
      Serial.print("N.Meas :");Serial.print(num_meas);Serial.print("\t");
      send_ttn_message(dataBuffer, COUNT_OF(dataBuffer));
      tilt_crit[0] = 0.0; tilt_crit[1] = 0.0;
    }
    // Housekeeping --------------------------------------------------------
    delay_smart(global_delay);
    i++; // increment while counter
    num_meas++; // increment total counter
    if (num_meas>4096){
      num_meas = 1;}
  } // end while
}
