
#include <Zumo32U4Motors.h>
#include <Zumo32U4Encoders.h>

#include "button.h"       //include your button class from last week
#include <event_timer.h>  //include your shiny, new event timer class
#include "segments.h"
#include <Robot.h>
#include "params.h"
#include "serial_comms.h"

volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Robot robot;
Button buttonA(14); //button A is pin 14 on the Zumo
EventTimer timer;   //assumes you named your class EventTimer

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

void setup()
{
  Serial.begin(115200);
  while(!Serial) {}  //IF YOU DON'T COMMENT THIS OUT, YOU MUST OPEN THE SERIAL MONITOR TO START
  Serial.println("Hi.");

  noInterrupts(); //disable interupts while we mess with the Timer4 registers

  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
  TCCR4D = 0x00; //normal mode

  OCR4C = 141;   //TOP goes in OCR4C
  TIMSK4 = 0x04; //enable overflow interrupt

  interrupts(); //re-enable interrupts    prevError = errorLeft;

  buttonA.Init(); //don't forget to call Init()!

  segments[0].leftSpeed = 0;
  segments[0].rightSpeed = 0;
  segments[0].duration = 10000;

  segments[1].leftSpeed = 59;
  segments[1].rightSpeed = 59;
  segments[1].duration = 3000;

  segments[2].leftSpeed = 59;
  segments[2].rightSpeed = -59;
  segments[2].duration = 410;

  segments[3].leftSpeed = 59;
  segments[3].rightSpeed = 59;
  segments[3].duration = 1000;
  //pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!
}

void loop()
{
  // handle state machine
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

  // instead of setting motors to segment speed, set target speed to segement speed and let PID controller set speed
  targetLeft = segments[currentSeg].leftSpeed;
  targetRight = segments[currentSeg].rightSpeed;
  robot.prevSegment = currentSeg;

  // handle PID
  if(readyToPID) //timer flag set
  {
    //clear the timer flag
    readyToPID = 0;

    //for tracking previous counts
    static int16_t prevLeft = 0;
    static int16_t prevRight = 0;

    //error sum
    static int16_t sumLeft = 0;
    static int16_t sumRight = 0;

    /*
     * Do PID stuffs here. Note that we turn off interupts while we read countsLeft/Right
     * so that it won't get accidentally updated (in the ISR) while we're reading it.
     */
    noInterrupts();
    int16_t speedLeft = countsLeft - prevLeft;
    int16_t speedRight = countsRight - prevRight;

    prevLeft = countsLeft;
    prevRight = countsRight;
    interrupts();

    float errorLeft = targetLeft - speedLeft;
    float errorRight = targetRight - speedRight;

    // convert ticks/sec to effort
    errorLeft = errorLeft / 75 * 400;
    errorRight = errorRight / 75 * 400;

    if (Ki > 0) { // fix initial jerk by only accumulating error if Ki term exists
      sumLeft += errorLeft;
      sumRight += errorRight;
    }

    float effortLeft = Kp * errorLeft + Ki * sumLeft;
    float effortRight = Kp * errorRight + Ki * sumRight;

    motors.setSpeeds(effortLeft, effortRight);

  }

}

/*
 * ISR for timing. Basically, raise a flag on overflow. Timer4 is set up to run with a pre-scaler
 * of 1024 and TOP is set to 249. Clock is 16 MHz, so interval is dT = (1024 * 250) / 16 MHz = 16 ms.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = 1;
}
