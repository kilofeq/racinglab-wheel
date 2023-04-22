#include "TorqueModbus.h"
#include <ModbusMaster.h>

TorqueModbus::TorqueModbus() {
}

TorqueModbus::~TorqueModbus() {
}

void TorqueModbus::setTorque(int16_t force, ModbusMaster modbus) {
	modbus.writeSingleRegister(200, force);
}
