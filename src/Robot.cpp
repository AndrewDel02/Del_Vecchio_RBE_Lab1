#include <Arduino.h>
#include <Robot.h>

Robot::Robot() {
  currentSegment = 0;
  prevSegment = 0;
  currentRobotState = IDLE;
}

void Robot::handleButtonPress() {
    switch (currentRobotState) {
      case IDLE:
      currentRobotState = DRIVING;
      currentSegment++;
      break;

      case DRIVING: // if button pushed while driving stop motors
      currentRobotState = IDLE;
      currentSegment = 0;
      break;
    }
}

void Robot::handleTimer() {
    if (currentSegment == 3) {
      currentRobotState = IDLE;
      currentSegment = 0;
    } else {
      currentSegment++;
    }
}

int Robot::getCurrentState() {
  switch (currentRobotState) {
    case IDLE: return 0;
    case DRIVING: return 1;
    default: return 2; // something is very wrong
  }
}
