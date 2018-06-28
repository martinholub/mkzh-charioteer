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
    if(i%accel_per==0){
      Serial.print("i:");Serial.print(num_meas);
      Serial.print(":Grav. Accel.: X="); //So that the display text ' X ='
      Serial.print(Xg);
      Serial.print("\t Y=");
      Serial.print(Yg);
      Serial.print("\t Z=");
      Serial.println(Zg);
    } // end if accel

    // alarm
    if((abs(Xg)> grav_thresh) || (abs(Yg) > grav_thresh)){
      do_alert = true;
    }
}

void update_temphum() {
	
	int chk = DHT11.read(DHT11PIN);
    float hm = (float)DHT11.humidity;
    float tm = (float)DHT11.temperature;
    
    // Serial.print("Humidity:");
    // Serial.print(hm, 2);
    // Serial.print(" % ");
    // Serial.print("\t Temp:    ");// Print a message of "Temp: "to the LCD.
    // Serial.print(tm, 2);// Print a centigrade temperature to the LCD. 
    // Serial.println(" C "); // Print the unit of the centigrade temperature to the LCD.
    
    // alarm
    bool ct = (tm<th_thresh[0])||(tm>th_thresh[1]);
    bool ch = (hm<th_thresh[2])||(hm>th_thresh[3]);
    if (ch || ct){
      do_alert = true;
    }
    // Running Average of Temperature
    myRA_T.addValue(tm); // Add value to buffer on each measurement
    myRA_h.addValue(hm);
    // Report Values --------------------------------------------------------------
    if(i%temp_per==0){
      if (myRA_T.getCount()>=myRA_T.getSize()){ // T+h measured together, cond. on one
        // Debug
        // Serial.print("Count:");Serial.print(myRA_T.getCount());
        // Serial.print("\t Size:");Serial.println(myRA_T.getSize());
        
        Serial.print("Rolling Averages:\t");
        Serial.print("Humidity:");
        Serial.print(myRA_h.getAverage(), 2);
        Serial.print(" % ");
        Serial.print("\t Temp:    ");
        Serial.print(myRA_T.getAverage(), 2);// Print a centigrade temperature 
        Serial.println(" C "); // Print the unit of the centigrade temperature
      }
    } // end if temp
}

void update_us() {
	
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
}