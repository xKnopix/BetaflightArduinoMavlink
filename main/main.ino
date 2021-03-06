// Example Code to read acceleration data from pixhawk
// Check HardwareSerial connection for your Arduino board and connect them according to the connection shown in guide.md

#include "BetaflightArduinoMavlink.h"
#include <HardwareSerial.h>

HardwareSerial &hs = Serial2;
BetaflightArduinoMavlink mav(hs);

void setup(){
  Serial.begin(57600);
  while(!mav.begin()){
    Serial.println("Not Connected!");
    delay(1000);
  }
  mav.Stream();
  delay(2000);
}

void loop(){
  float xacc, yacc, zacc, gps_lat;

  mav.ReadAcceleration(&xacc, &yacc, &zacc);
  Serial.print("X: "); Serial.print(xacc);
  Serial.print("  Y: "); Serial.print(yacc);
  Serial.print("  Z: "); Serial.println(zacc);

  delay(100);
}
