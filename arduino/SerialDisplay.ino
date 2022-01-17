/*

 The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K potentiometer:
 * ends to +5V and ground
 * wiper to LCD VO pin

*/

#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
int input_serial;


#define SIZE 16

int previous[16];  // the array 
byte list_end = 0 ; // the end pointer

// A function to append - it may fail if the array is full, but returns a boolean to indicate if it worked.
bool append_list (int element)
{
  if (list_end < SIZE)
  {
    previous[list_end++] = element ;
    return true;
  }
  return false;
}


void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);
  for (byte i = 0; i < 16; i = i + 1) {
    previous[i] = " ";
  }
}

void loop() {
  if (Serial.available()) {
    delay(10);
    lcd.clear();
    
    lcd.setCursor(0, 0);
    
    for (byte i = 0; i < 16; i = i + 1) {
      lcd.write(previous[i]);
      previous[i] = " ";
      list_end = 0 ;
    }
    lcd.setCursor(0, 1);
    while (Serial.available() > 0) {
      input_serial = (Serial.read());
      append_list(input_serial);
      lcd.write(input_serial);
      
    }
  }
}
