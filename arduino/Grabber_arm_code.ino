#include <Servo.h>

Servo Grab;
Servo Flip;

const int buttonPin = 9;
int buttonState = 0;
int Pos = 100;
int PosGrab = 0;
int PosFlip = 0;

void setup() {
  Serial.begin(9600);


  
  pinMode(buttonPin, INPUT);
  Flip.attach(8);
  Grab.attach(7);
//  Pos = calibrate();

}

int calibrate() {
  for (PosGrab = 40; PosGrab >= 0; PosGrab -= 1) { 
    Grab.write(PosGrab);
    buttonState = digitalRead(buttonPin);
    if  (buttonState == HIGH) {
      Serial.println(PosGrab);
      return (PosGrab);
      break;
    }
   delay(30);
  }
}

void GrabCan() {
  Grab.write(Pos+70);
  Flip.write(180);
  Pos = calibrate();
  Flip.write(0);
    
}

void loop() {
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == 'A') {
      GrabCan();
     }
  }
}
