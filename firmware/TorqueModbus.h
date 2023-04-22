#include <Arduino.h>
#include <ModbusMaster.h>


class TorqueModbus {
 public:
  TorqueModbus(void);
  ~TorqueModbus(void);
   void setTorque(int16_t force, ModbusMaster modbus);
};
