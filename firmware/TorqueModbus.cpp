#include "TorqueModbus.h"
#include <ModbusMaster.h>

TorqueModbus::TorqueModbus() {
}

TorqueModbus::~TorqueModbus() {
}

void TorqueModbus::setTorque(int16_t force, ModbusMaster modbus) {
	int normalizedForce = map(force, -255,255, MINFORCE, MAXFORCE);
	modbus.writeSingleRegister(200, normalizedForce);
}
