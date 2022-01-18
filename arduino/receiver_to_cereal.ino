

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

// First Example in a series of posts illustrating reading an RC Receiver with
// micro controller interrupts.
//
// Subsequent posts will provide enhancements required for real world operation
// in high speed applications with multiple inputs.
//
// http://rcarduino.blogspot.com/
//
// Posts in the series will be titled - How To Read an RC Receiver With A Microcontroller

// See also http://rcarduino.blogspot.co.uk/2012/04/how-to-read-multiple-rc-channels-draft.html

#define NEUTRAL_POSITION 1500

#define SWITCH_SIGNAL_IN 16
#define SWITCH_SIGNAL_IN_PIN 0
volatile int nSwitchIn = NEUTRAL_POSITION;
volatile unsigned long ulSStartPeriod = 0;
volatile boolean bNewSwitchSignal = false;

#define BIND_SIGNAL_IN 21
#define BIND_SIGNAL_IN_PIN 5
volatile int nBindIn = NEUTRAL_POSITION;
volatile unsigned long ulBStartPeriod = 0;
volatile boolean bNewBindSignal = false;

#define THROTTLE_SIGNAL_IN 22
#define THROTTLE_SIGNAL_IN_PIN 6
volatile int nThrottleIn = NEUTRAL_POSITION;
volatile unsigned long ulTStartPeriod = 0;
volatile boolean bNewThrottleSignal = false;

#define AILERON_SIGNAL_IN 18
#define AILERON_SIGNAL_IN_PIN 2
volatile int nAileronIn = NEUTRAL_POSITION;
volatile unsigned long ulAStartPeriod = 0;
volatile boolean bNewAileronSignal = false;

#define ELEVATOR_SIGNAL_IN 19
#define ELEVATOR_SIGNAL_IN_PIN 3
volatile int nElevatorIn = NEUTRAL_POSITION;
volatile unsigned long ulEStartPeriod = 0;
volatile boolean bNewElevatorSignal = false;

#define RUDDER_SIGNAL_IN 20
#define RUDDER_SIGNAL_IN_PIN 4
volatile int nRudderln = NEUTRAL_POSITION;
volatile unsigned long ulRStartPeriod = 0;
volatile boolean bNewRudderSignal = false;

int inPin = 13;

void setup()
{
  pinMode(inPin, INPUT);
  attachPinChangeInterrupt(ELEVATOR_SIGNAL_IN, calcElevator, HIGH);
  attachPinChangeInterrupt(AILERON_SIGNAL_IN, calcAileron, HIGH);
  attachPinChangeInterrupt(RUDDER_SIGNAL_IN, calcRudder, HIGH);
  attachPinChangeInterrupt(THROTTLE_SIGNAL_IN, calcThrottle, HIGH);
  attachPinChangeInterrupt(SWITCH_SIGNAL_IN, calcSwitch, HIGH);
  attachPinChangeInterrupt(BIND_SIGNAL_IN, calcBind, HIGH);
  Serial.begin(9600);
}

void loop()
{
  if (bNewAileronSignal)
  {
    Serial.print(nAileronIn);
    Serial.print(",");
    bNewAileronSignal = false;
  }
  if (bNewElevatorSignal)
  {
    Serial.print(nElevatorIn);
    Serial.print(",");
    bNewElevatorSignal = false;
  }
  if (bNewRudderSignal)
  {
    Serial.print(nRudderln);
    Serial.print(",");
    bNewRudderSignal = false;
  }
  if (bNewSwitchSignal)
  {
    Serial.print(nSwitchIn);
    Serial.print(",");
    bNewSwitchSignal = false;
  }
  if (bNewBindSignal)
  {
    Serial.print(nBindIn);
    Serial.print(",");
    bNewBindSignal = false;
  }
  if (bNewThrottleSignal)
  {
    Serial.print(nThrottleIn);
    Serial.print(",");
    bNewThrottleSignal = false;
  }
  Serial.println(digitalRead(inPin));
  delay(100);
}

void calcBind()
{
  if (digitalRead(BIND_SIGNAL_IN_PIN) == HIGH)
  {
    ulBStartPeriod = micros();
  }
  else
  {
    if (ulBStartPeriod && (bNewBindSignal == false))
    {
      nBindIn = (int)(micros() - ulBStartPeriod);
      ulBStartPeriod = 0;
      bNewBindSignal = true;
    }
  }
}

void calcSwitch()
{
  if (digitalRead(SWITCH_SIGNAL_IN_PIN) == HIGH)
  {
    ulSStartPeriod = micros();
  }
  else
  {
    if (ulSStartPeriod && (bNewSwitchSignal == false))
    {
      nSwitchIn = (int)(micros() - ulSStartPeriod);
      ulSStartPeriod = 0;
      bNewSwitchSignal = true;
    }
  }
}

void calcElevator()
{
  if (digitalRead(ELEVATOR_SIGNAL_IN_PIN) == HIGH)
  {
    ulEStartPeriod = micros();
  }
  else
  {
    if (ulEStartPeriod && (bNewElevatorSignal == false))
    {
      nElevatorIn = (int)(micros() - ulEStartPeriod);
      ulEStartPeriod = 0;
      bNewElevatorSignal = true;
    }
  }
}

void calcAileron()
{
  if (digitalRead(AILERON_SIGNAL_IN_PIN) == HIGH)
  {
    ulAStartPeriod = micros();
  }
  else
  {
    if (ulAStartPeriod && (bNewAileronSignal == false))
    {
      nAileronIn = (int)(micros() - ulAStartPeriod);
      ulAStartPeriod = 0;
      bNewAileronSignal = true;
    }
  }
}

void calcRudder()
{
  if (digitalRead(RUDDER_SIGNAL_IN_PIN) == HIGH)
  {
    ulRStartPeriod = micros();
  }
  else
  {
    if (ulRStartPeriod && (bNewRudderSignal == false))
    {
      nRudderln = (int)(micros() - ulRStartPeriod);
      ulRStartPeriod = 0;
      bNewRudderSignal = true;
    }
  }
}

void calcThrottle()
{
  if (digitalRead(THROTTLE_SIGNAL_IN_PIN) == HIGH)
  {
    ulTStartPeriod = micros();
  }
  else
  {
    if (ulTStartPeriod && (bNewThrottleSignal == false))
    {
      nThrottleIn = (int)(micros() - ulTStartPeriod);
      ulTStartPeriod = 0;
      bNewThrottleSignal = true;
    }
  }
}
