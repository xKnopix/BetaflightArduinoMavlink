#ifndef BetaflightArduinoMavlink_h
#define BetaflightArduinoMavlink_h

#include "MavLinkHelpHeaders/src/common/mavlink.h"
#include "MavLinkHelpHeaders/src/checksum.h"
#include "MavLinkHelpHeaders/src/mavlink_types.h"
#include "MavLinkHelpHeaders/src/protocol.h"
#include <Arduino.h>
#include <HardwareSerial.h>

class BetaflightArduinoMavlink
{
  public:
    BetaflightArduinoMavlink(HardwareSerial &hs);
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
