#include <Servo.h>

Servo Grab;
Servo Flip;

const int buttonPin = 13;
const int FlipPin = 12;
const int GrabPin = 11;
int buttonState = 0;
int Pos = 100;
int PosGrab = 0;
int PosFlip = 0;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  Flip.attach(FlipPin);
  Grab.attach(GrabPin);
  Pos = calibrate();
  Flip.write(7);
}

int calibrate() {
  for (PosGrab = 40; PosGrab >= 0; PosGrab -= 1) { 
    Grab.write(PosGrab);
    buttonState = digitalRead(buttonPin);
    if  (buttonState == HIGH) {
      Serial.println("AAA");
      return (PosGrab);
      break;
    }
   delay(30);
  }
}

void FlipCan(int startPoint, int endPoint, int stepSize) {
  for (PosFlip = startPoint; PosFlip == endPoint; PosFlip -= stepSize) { 
    Flip.write(PosFlip);
    delay(15);
  }
}

void GrabCan() {
  Grab.write(Pos+70);
  delay(200);
  for (PosFlip = 7; PosFlip <= 180; PosFlip += 2) { 
    Flip.write(PosFlip);
    delay(10);
  }
  Pos = calibrate();
  Grab.write(Pos);
  for (PosFlip = 180; PosFlip >= 7; PosFlip -= 2) { 
    Flip.write(PosFlip);
    delay(10);
  }
}

void loop() {
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == 'A') {
      GrabCan();
     }
  }
}
