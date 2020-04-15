#include <Arduino.h>
#include <Zumo32U4Motors.h>
#include <Zumo32U4Encoders.h>
#include <Wire.h>
#include "button.h"       //include your button class from last week
#include <event_timer.h>  //include your shiny, new event timer class
#include "segments.h"
#include <Robot.h>

Robot robot;
Button buttonA(14); //button A is pin 14 on the Zumo
EventTimer timer;   //assumes you named your class EventTimer

//use the Pololu libraries for motors and encoders
Zumo32U4Motors motors;
Zumo32U4Encoders encoders; //(we're not acutally using this in this code, but we will soon)

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello.");

  buttonA.Init(); //don't forget to call Init()!

  segments[0].leftSpeed = 0;
  segments[0].rightSpeed = 0;
  segments[0].duration = 10000;

  segments[1].leftSpeed = 223;
  segments[1].rightSpeed = 200;
  segments[1].duration = 3000;

  segments[2].leftSpeed = 223;
  segments[2].rightSpeed = -200;
  segments[2].duration = 410;

  segments[3].leftSpeed = 223;
  segments[3].rightSpeed = 200;
  segments[3].duration = 2500;
}

void loop()
{
  //put in a simple event-driven structure
  //if(SomeEvent()) HandleSomeEvent()
  //...
  int currentSeg = robot.currentSegment;

  if (buttonA.CheckButtonPress()) {
    Serial.println("button");
    Serial.println(robot.getCurrentState());
    robot.handleButtonPress();
    Serial.println(robot.getCurrentState());
  }

  if (timer.CheckExpired()) {
    Serial.println(currentSeg);
    robot.handleTimer();
    timer.Cancel();
  }

  if (robot.prevSegment != currentSeg && currentSeg != 0) { // set timer on new segment
    timer.Start(segments[currentSeg].duration);
  }
  motors.setLeftSpeed(segments[currentSeg].leftSpeed);
  motors.setRightSpeed(segments[currentSeg].rightSpeed);
  robot.prevSegment = currentSeg;
};
