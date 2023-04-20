#include <Arduino.h>
#include <ModbusMaster.h>
#define MAXFORCE 100
#define MINFORCE -100


class TorqueModbus {
 public:
  TorqueModbus(void);
  ~TorqueModbus(void);
   void setTorque(int16_t force, ModbusMaster modbus);
};
