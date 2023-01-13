#include <SoftwareSerial.h>

#define RS485RX               18
#define RS485TX               19

#define sensorFrameSize       9
#define sensorWaitingTime     1000

#define sensorID              0x01
#define sensorFunction        0x04
#define sensorByteResponse    0x04

float volts, current, watts;

SoftwareSerial sensor(RS485RX, RS485TX);

unsigned char byteRequest[8] = {0x01,0x04,0x00,0x00,0x00,0x02,0x71,0xCB};
unsigned char byteResponse[9] = {};

void setup() {
  Serial.begin(115200);
  sensor.begin(9600);

  Serial.println("\nHibiscus Sense RS485 EASTRON SMART X96-5 & MQTT");

  Serial.println();
}

void loop() {
  sensor.write(byteRequest, 8);

  unsigned long resptime = millis();
  while ((sensor.available() < sensorFrameSize) && ((millis() - resptime) < sensorWaitingTime)) {
    delay(1);
  }

  while (sensor.available()) {
    for (int n = 0; n < sensorFrameSize; n++) {
      byteResponse[n] = sensor.read();
    }

    if (byteResponse[0] != sensorID && byteResponse[1] != sensorFunction && byteResponse[2] != sensorByteResponse) {
      
    }
  }

  volts = ieee754((float)byteResponse[3], (float)byteResponse[4], (float)byteResponse[5], (float)byteResponse[6]);
  
  Serial.println("Volts: " + (String)volts + " V");
  

  Serial.println();

  delay(1000);
}

float ieee754(float byte1, float byte2, float byte3, float byte4) {
    unsigned char data[4] = {byte1, byte2, byte3, byte4};
    
    // Extract the sign, exponent, and mantissa from the data
    int sign = (data[0] >> 7) & 1;
    int exponent = ((data[0] & 0x7F) << 1) | ((data[1] >> 7) & 1);
    int mantissa = ((data[1] & 0x7F) << 16) | (data[2] << 8) | data[3];

    // Handle special cases (zero, infinity, and NaN)
    if (exponent == 0 && mantissa == 0) {
        return sign ? -0.0 : 0.0;
    }
    if (exponent == 0xFF && mantissa != 0) {
        return sign ? -NAN : NAN;
    }
    if (exponent == 0xFF && mantissa == 0) {
        return sign ? -INFINITY : INFINITY;
    }

    // Calculate the actual floating-point value
    float value = powf(2, exponent - 127);
    value *= (1 + (float)mantissa / 8388608);
    if (sign) {
        value = -value;
    }

    return value;
}
