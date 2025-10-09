#include "ports.h"
#include "robot.h"

const int32_t Ports::DRIVE_TOP_LEFT_MOTOR = vex::PORT4;
const int32_t Ports::DRIVE_MID_LEFT_MOTOR = vex::PORT6;
const int32_t Ports::DRIVE_BOT_LEFT_MOTOR = vex::PORT5;
const int32_t Ports::DRIVE_TOP_RIGHT_MOTOR = vex::PORT1;
const int32_t Ports::DRIVE_MID_RIGHT_MOTOR = vex::PORT3;
const int32_t Ports::DRIVE_BOT_RIGHT_MOTOR = vex::PORT2;

// TODO: find true values
vex::triport::port Ports::DRIVE_LEFT_ENCODER = Robot::getBrain().ThreeWirePort.A;
vex::triport::port Ports::DRIVE_RIGHT_ENCODER = Robot::getBrain().ThreeWirePort.B;
vex::triport::port Ports::DRIVE_BOTTOM_ENCODER = Robot::getBrain().ThreeWirePort.C;