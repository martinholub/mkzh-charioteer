# Follow-up notes for meeting #1

![](meeting1a.jpg)
![](meeting1b.jpg)

# Features

## Communication (Martin)

### Questions

+ Can GPS talk from a box? Where it needs to be?
+ What sensors does the FMLR TrackerTwo have?
+ Have safe is the data on transmission uplink/downlink?
+ How often do we need to transmit data? How often we can? (LoRaWAN)

## Hardware (Reto)

### Accelerometer
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor?view=all
https://www.aliexpress.com/item/BNO055-CJMCU-055-Intelligent-9-Axis-Attitude-Sensor-precision-Accelerometer-gyroscope-magnetic-sensor-module-for-arduino/32842626275.html
https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/
https://www.davidpilling.com/wiki/index.php/BNO055


#### Notes
To solder or not to solder [the barrel](https://forums.adafruit.com/viewtopic.php?f=19&t=118110)
> The object resembling a tilt switch is probably a 32.768 kHz quartz crystal. It's optional. You can send a command to the BNO telling it to use either its lower-accuracy internal oscillator, or its higher-accuracy crystal oscillator.

SDA/SCL pins on [Leonardo and Uno](https://arduino.stackexchange.com/a/160)

## Sensing (Antonias)

## Dataplatform (...)

console.thethingsnetwork.org
platform.makezurich.ch