#include <ModbusMaster.h>
#include "Joystick.h"

#define SLAVE_ID 1
#define MAX485_DE 3
#define MAX485_RE_NEG 2
#define SLAVE_BAUDRATE 115200
#define ENCODER_CPR 10000
#define WHEEL_DEGREES 1080
#define MAX_TORQUE 100
#define ENDSTOP_TORQUE_GAIN 30

float wheel_turns = (float)WHEEL_DEGREES / (float)360;
float joystick_max_position = wheel_turns * float(ENCODER_CPR);

//X-axis & Y-axis REQUIRED
Joystick_ Joystick(2, 
  JOYSTICK_TYPE_JOYSTICK, 4, 0,
  true, true, false, //X,Y, noZ
  false, false, false,//Rx,Ry,Rz
  false, false, false, false, false);

Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};

ModbusMaster modbus;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}


void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(9600);
  Serial1.begin(SLAVE_BAUDRATE, SERIAL_8O1);
  modbus.begin(SLAVE_ID, Serial1);

  // Callbacks allow us to configure the RS485 transceiver correctly
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);

  // FFB
  Joystick.setXAxisRange(-joystick_max_position, joystick_max_position);
  mygains[0].totalGain = 100;//0-100
  mygains[0].springGain = 100;//0-100
  Joystick.setGains(mygains);
  Joystick.begin();
}

int prev_encoder_value;
int prev_torque = 0;
int x_axis_position = 0;

void loop()
{
  uint8_t result;
  uint16_t data[6];
  int torque_setting_position = 200;
  int encoder_setting_position = 391;

  // Encoder
  result = modbus.readHoldingRegisters(encoder_setting_position, 1);
  if (result == modbus.ku8MBSuccess)
  {
    int encoder_value = modbus.getResponseBuffer(0);
    if (encoder_value != prev_encoder_value) {
      int encoder_position_change = encoder_value - prev_encoder_value;
      // Handle new rotation
      if (encoder_position_change > 9000) {
        int correct_encoder_position_change = ENCODER_CPR - encoder_position_change;
        x_axis_position = x_axis_position + correct_encoder_position_change;
      } else if (encoder_position_change < -9000) {
        int correct_encoder_position_change = ENCODER_CPR + encoder_position_change;
        x_axis_position = x_axis_position + correct_encoder_position_change;
      } else {
        x_axis_position = x_axis_position + encoder_position_change;
      }
      prev_encoder_value = encoder_value;
    }
  }

  // FFB
  int torque = 0;
  // Endstop
  int x_axis_offset =  x_axis_position - joystick_max_position;
  if (x_axis_offset < 0) {
    x_axis_offset = x_axis_offset * -1;
  }
  if (x_axis_position > joystick_max_position) {
    torque = -ENDSTOP_TORQUE_GAIN * x_axis_offset;
    if (torque > MAX_TORQUE) {
      torque = MAX_TORQUE;
    }
  } else if (x_axis_position < -joystick_max_position) {
    torque = ENDSTOP_TORQUE_GAIN * x_axis_offset;
    if (torque < -MAX_TORQUE) {
      torque = -MAX_TORQUE;
    }
  }
  myeffectparams[0].springMaxPosition = joystick_max_position;
  myeffectparams[0].springPosition = x_axis_position;
  Joystick.setXAxis(x_axis_position);
  Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);
  if (x_axis_position < joystick_max_position && x_axis_position > -joystick_max_position) {
    torque = forces[0] / 255;
  }
  if (prev_torque != torque) {
    modbus.writeSingleRegister(torque_setting_position, torque);
  }
  prev_torque = torque;
}