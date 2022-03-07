#ifndef PixhawkArduinoMAVLink_h
#define PixhawkArduinoMAVLink_h

#include "PixhawkArduinoMAVLink-master/src/common/mavlink.h"
#include "PixhawkArduinoMAVLink-master/src/checksum.h"
#include "PixhawkArduinoMAVLink-master/src/mavlink_types.h"
#include "PixhawkArduinoMAVLink-master/src/protocol.h"
#include <Arduino.h>
#include <HardwareSerial.h>

class PixhawkArduinoMAVLink
{
  public:
    PixhawkArduinoMAVLink(HardwareSerial &hs);
    bool begin();
    void ReadAcceleration(float *xacc, float *yacc, float *zacc);
    void Stream();
  private:
    HardwareSerial* _MAVSerial;
    double MILLIG_TO_MS2;
    uint8_t system_id;
    uint8_t component_id;
    uint8_t type;
    uint8_t autopilot;
    uint8_t received_sysid; // Pixhawk sysid
    uint8_t received_compid; // Pixhawk compid
};

#endif
