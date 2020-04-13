#include <Arduino.h>

class Robot {
  enum robotStates {DRIVING, IDLE} currentRobotState;

public:
  int currentSegment;
  int prevSegment;
  Robot();
  void handleButtonPress();
  void handleTimer();
  int getCurrentState();
};
