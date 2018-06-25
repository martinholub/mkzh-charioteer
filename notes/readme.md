# Follow-up notes for meeting #1

![](meeting1a.jpg)
![](meeting1b.jpg)

# Features

## Communication (Martin)

TODO: suck out info from here: https://www.thethingsnetwork.org/docs/lorawan/

#### Requirements
Note: sensors have errors - can [round](https://www.thethingsnetwork.org/docs/devices/bytes.html#2-round) and save on data transmission
+ Tilt: 
  + 1/0 ... Ok/Bad
  + 0.xx for X and Y axes -> 2 bytes
+ Temp:
  + 1/0 ... Ok/Bad
  + xx.x ... float with 1 decimal -> 1 byte (?)
+ Humidity:
  + 1/0 ... Ok/Bad -> 1 bit
  + xx %  -> 1 byte
+ US sensor:
  + xx for 3 axes -> 3 bytes
+ [GPS coords](https://www.thethingsnetwork.org/forum/t/best-practices-when-sending-gps-location-data/1242):
  + see the link for detailed limitations and tips

#### Limitations
+ [Limitations](https://www.thethingsnetwork.org/docs/lorawan/limitations.html)

[Limitations: data rate, packet size, 30 seconds uplink and 10 messages downlink per day Fair Access Policy](https://www.thethingsnetwork.org/forum/t/limitations-data-rate-packet-size-30-seconds-uplink-and-10-messages-downlink-per-day-fair-access-policy/1300)
  + 30 seconds air-time per device per day. For 10 bytes of payload, this translates in (approx.):
    + 20 messages per day at SF12
    + 500 messages per day at SF7
  + Downlink bandwidth is even more restricted (10 msg per day) = you can’t send all messages as ‘confirmed uplink’
  - Payload should be as small as possible. A good goal is to keep it under 12 bytes. ...
  - Interval between messages should be in the range of several minutes, ...
If your application requires more bandwidth, think of another solution
  + In short LoRa excels at rare and small messages from a device that can live independetly for years. Does this fit our application?
  
#### The Network
> Each transmission blocks the receivers of a gateway for some time, it is better to send messages in a shorter time [source](https://www.thethingsnetwork.org/docs/network/architecture.html#downlink-configuration-router).

> LoRaWAN is a long-range radio protocol, making it likely that the message is received by more than one gateway [source](https://www.thethingsnetwork.org/docs/network/architecture.html#de-duplication-broker). Backend then must perform some de-duplication to obtain unique message. The information from duplictaes can be used to get location of the emitting device. Required de-duplication time is about 200ms.

> The frame counter in LoRaWAN messages is a security measure used to detect replay attacks. After validating the MIC, the Broker checks if the Frame counter is valid. As frame counters can only increase, a message with a frame counter that is lower than the last known frame counter should be dropped. [sourcce](https://www.thethingsnetwork.org/docs/network/architecture.html#frame-counter-check-broker)

`decoder`, `converter`, `validator` functions are used [to interpret](https://www.thethingsnetwork.org/docs/network/architecture.html#payload-conversion-handler) message on the handler side.

#### Security
> LoRaWAN enforces using AES 128-bit message integrity checks and payload encryption. Payloads are fully encrypted between the Node and the Handler component of the backend. This means you can choose to operate your own private Handler and have real end-to-end encryption. [source](https://www.thethingsnetwork.org/docs/network/security.html). The metadata is not encrypted.

#### Devices

> Technically, you can send 51 bytes. But, the more bytes you’ll send, the more airtime the package will cost you. [source](https://www.thethingsnetwork.org/docs/devices/bytes.html#how-many-bytes-can-i-send)

[How to send decimals?](https://www.thethingsnetwork.org/docs/devices/bytes.html#how-to-send-decimals)
[How to send multiple numbers?](https://www.thethingsnetwork.org/docs/devices/bytes.html#how-to-send-multiple-numbers)
[How to send big numbers?](https://www.thethingsnetwork.org/docs/devices/bytes.html#how-to-send-big-numbers)
How to send text? The short answer is: don’t.


> Devices should perform join operations the less possible in their lifetime. The LoRaWAN specifications warn especially against systematic rejoin in case of network failure. A device should keep the result of an activation in permanent storage if the device is expected to be turned off during its lifetime. [source](https://www.thethingsnetwork.org/docs/devices/bestpractices.html#otaa-best-practices)


#### Practical Bits and Pieces


Transmitted data:
  + Can get [some metadata](https://www.thethingsnetwork.org/docs/network/architecture.html#gateway-protocol-translation-routerbridge) from gateway message:
    + signal strength (RSSI) and signal-to-noise ratio (SNR)
  + Message has "port" field. Can use to switch between incoming measurements
  
  
+ Neither LoRa nor GPS will be able to communicate from a box
  + already indoor coverage is a problem
  + we may want to prototype from inside a box?
  + No guarantee on acceptance of message

+ Should be able to switch network providers
  + e.g. tnn > fallback to loriot > fallback to swisscom
  + provider may limit number of messages (Swisscom... 10/day)
  + this may be possible from platform.makezurich.ch
  
### Questions
+ Can GPS talk from a box? Where it needs to be?
+ What sensors does the FMLR TrackerTwo have?
+ How often do we need to transmit data? How often we can? (LoRaWAN)

### References:

[TTN-Learn-Network](https://www.thethingsnetwork.org/docs/network/architecture.html)

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

https://learn.adafruit.com/adxl345-digital-accelerometer/programming


## Sensing (Antonias)

## Dataplatform (Charilaos)

console.thethingsnetwork.org
platform.makezurich.ch