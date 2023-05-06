#include <ModbusMaster.h>
#include "Joystick.h"

#define SLAVE_ID 1
#define MAX485_DE 3
#define MAX485_RE_NEG 2
#define SLAVE_BAUDRATE 115200
#define ENCODER_CPR 10000
#define WHEEL_DEGREES 1080
#define END_STOP_TORQUE 100
#define frictionMaxPositionChangeCfg 25
#define inertiaMaxAccelerationCfg 10
#define damperMaxVelocityCfg 150

float wheel_turns = (float)WHEEL_DEGREES / (float)360;
float joystick_max_position = (wheel_turns * float(ENCODER_CPR)) /2;
int encoder_min = 0;
int encoder_max = ENCODER_CPR - 1;

bool posUpdated = false;
unsigned long lastEffectsUpdate;
unsigned long nextJoystickMillis;
unsigned long nextEffectsMillis;
int lastX;
int lastVelX;
int lastAccelX;

//X-axis & Y-axis REQUIRED
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
JOYSTICK_TYPE_JOYSTICK, 8, 0,
true, true, false, //X,Y, noZ
false, false, false,//Rx,Ry,Rz
false, false, false, false, false);

Gains mygains[2];
EffectParams effects[2];
int32_t forces[2] = {0};

ModbusMaster modbus;

int torque_setting_position = 200;
int encoder_setting_position = 391;
int turns = 0;
int prev_encoder_value;
int prev_torque = 0;
int x_axis_position = 0;
int encoder_center_position = 0;

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
  mygains[0].totalGain = 50;//0-100
  mygains[0].springGain = 100;//0-100
  Joystick.setGains(mygains);
  Joystick.begin();
  // Inital encoder value
  uint8_t result = modbus.readHoldingRegisters(encoder_setting_position, 1);
  if (result == modbus.ku8MBSuccess)
  {
    int encoder_value = modbus.getResponseBuffer(0);
    encoder_center_position = encoder_value; 
    prev_encoder_value = encoder_value;
  }
}

void updatePosition(){
  // Encoder
  uint8_t result = modbus.readHoldingRegisters(encoder_setting_position, 1);
  if (result == modbus.ku8MBSuccess)
  {
    int encoder_value = modbus.getResponseBuffer(0);
    int center_offset = encoder_value - encoder_center_position;
    if (center_offset < encoder_min) {
      encoder_value = encoder_max - -center_offset;
    } else {
      encoder_value = encoder_value - encoder_center_position;
    }
    int encoder_position_change = encoder_value - prev_encoder_value;
    if (encoder_value != prev_encoder_value) {
      posUpdated = true;
      // Handle new rotation
      if (encoder_position_change > 9000) {
        turns = turns - 1;
      } else if (encoder_position_change < -9000) {
        turns = turns + 1;
      }
      int negative_encoder_value = map(encoder_value, 0, encoder_max, -encoder_max, 0);
      if (turns > 0) {
        x_axis_position = ENCODER_CPR * turns + encoder_value;
      } else if (turns < -1) {
        int corrected_turns = turns + 1;
        x_axis_position = ENCODER_CPR * corrected_turns + negative_encoder_value;
      } else {
        if (turns == -1) {
          x_axis_position = negative_encoder_value;
        } else {
          x_axis_position = encoder_value;
        }
      }
      prev_encoder_value = encoder_value;
    }
  }
}

void loop()
{
  unsigned long currentMillis;
  currentMillis = millis();
  // do not run more frequently than these many milliseconds
  if (currentMillis >= nextJoystickMillis) {
    updatePosition();
    nextJoystickMillis = currentMillis + 2;
    // we calculate condition forces every 100ms or more frequently if we get position updates
    if (currentMillis >= nextEffectsMillis || posUpdated) {
      updateEffects(true);
      nextEffectsMillis = currentMillis + 100;
      posUpdated = false;
    } else {
      // calculate forces without recalculating condition forces
      // this helps having smoother spring/damper/friction
      // if our update rate matches our input device
      updateEffects(false);
    }
  }
  // FFB
  int torque = 0;
  effects[0].springMaxPosition = joystick_max_position;
  effects[0].springPosition = x_axis_position;
  Joystick.setXAxis(x_axis_position);
  Joystick.setEffectParams(effects);
  Joystick.getForce(forces);
  // Endstop
  if (x_axis_position > 0 && x_axis_position + 1 >= joystick_max_position) {
    torque = -END_STOP_TORQUE;
  } else if (x_axis_position < 0 && x_axis_position - 1 <= -joystick_max_position) {
    torque = END_STOP_TORQUE;
  } else {
    torque = forces[0];
  }
  if (prev_torque != torque) {
    modbus.writeSingleRegister(torque_setting_position, torque);
  }
  prev_torque = torque;
}

void updateEffects(bool recalculate){
    effects[0].frictionMaxPositionChange = frictionMaxPositionChangeCfg;
    effects[0].inertiaMaxAcceleration = inertiaMaxAccelerationCfg;
    effects[0].damperMaxVelocity = damperMaxVelocityCfg;
    effects[0].springMaxPosition = joystick_max_position;
    effects[0].springPosition = x_axis_position;

    unsigned long currentMillis;
    currentMillis = millis();
    int16_t diffTime = currentMillis - lastEffectsUpdate;

    if (diffTime > 0 && recalculate) {
        lastEffectsUpdate = currentMillis;
        int16_t positionChangeX = x_axis_position - lastX;
        int16_t velX = positionChangeX / diffTime;
        int16_t accelX = ((velX - lastVelX) * 10) / diffTime;
    
        effects[0].frictionPositionChange = velX;
        effects[0].inertiaAcceleration = accelX;
        effects[0].damperVelocity = velX;

        lastX = x_axis_position;
        lastVelX = velX;
        lastAccelX = accelX;
    } else {
        effects[0].frictionPositionChange = lastVelX;
        effects[0].inertiaAcceleration = lastAccelX;
        effects[0].damperVelocity = lastVelX;
    }

    Joystick.setEffectParams(effects);
    Joystick.getForce(forces);
    int torque = 0;
    if (x_axis_position > 0 && x_axis_position + 1 >= joystick_max_position) {
      torque = -END_STOP_TORQUE;
    } else if (x_axis_position < 0 && x_axis_position - 1 <= -joystick_max_position) {
      torque = END_STOP_TORQUE;
    } else {
      torque = forces[0];
    }
    if (prev_torque != torque) {
      modbus.writeSingleRegister(torque_setting_position, torque);
    }
    prev_torque = torque;
}


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
