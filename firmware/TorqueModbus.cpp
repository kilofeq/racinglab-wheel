#include "TorqueModbus.h"
#include <ModbusMaster.h>

TorqueModbus::TorqueModbus() {
}

TorqueModbus::~TorqueModbus() {
}

void TorqueModbus::setTorque(uint16_t torque, ModbusMaster modbus) {
	modbus.writeSingleRegister(200, torque);
}
