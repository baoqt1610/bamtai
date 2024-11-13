
#include "Arduino.h"
#include "OneButton.h"

#define PIN_INPUT1 0

// Setup a new OneButton on pin PIN_INPUT2.
OneButton button1(PIN_INPUT1, true);

// the LED
const int pwmpin = 1;
const int dcpin = 2;
int state = 0;
int tocdo = 100; 



// setup code here, to run once:
void setup() {

  pinMode(dcpin,OUTPUT);  
  pinMode(pwmpin,OUTPUT);  
  analogWrite(pwmpin, 0);
  digitalWrite (dcpin, 0);
  // Setup the Serial port. see http://arduino.cc/en/Serial/IfSerial
  // Serial.begin(115200);
  // while (!Serial) {
  //   ;  // wait for serial port to connect. Needed for Leonardo only
  // }
  //Serial.println("Starting TwoButtons...");

  // link the button 1 functions.
  button1.attachClick(click1);
  button1.attachLongPressStart(longPressStart1);


}  // setup


// main code here, to run repeatedly:
void loop() {
  // keep watching the push buttons:
  button1.tick();
//dieu khien

  // You can implement other code in here or just wait a while
  delay(10);
}  // loop


// ----- button 1 callback functions

// This function will be called when the button1 was pressed 1 time (and no 2. button press followed).
void click1() {
  //Serial.println("Button 1 click.");
if (state == 0)
    {
      state = 1;
      
    }

    else 
    {
      state = 0;

    }

  if (state == 0)

  {
    analogWrite(pwmpin, 0);
    digitalWrite (dcpin, 0);
   // Serial.println("Tat may");
  }
else 

{

  analogWrite(pwmpin, tocdo);
  digitalWrite (dcpin, 1);
  //Serial.println("Bat may");
  
}
}  // click1


// This function will be called when the button1 was pressed 2 times in a short timeframe.
void doubleclick1() {
  //Serial.println("Button 1 doubleclick.");
}  // doubleclick1


// This function will be called once, when the button1 is pressed for a long time.


// This function will be called often, while the button1 is pressed for a long time.
void longPressStart1() {
  //Serial.println("Button 1 longPress...");
  tocdo = tocdo + 50; 

  if (tocdo > 200)
  {tocdo = 50;} 
if (state ==1)
{
   analogWrite(pwmpin, tocdo);
}

}  // longPress1


// This function will be called once, when the button1 is released after beeing pressed for a long time.
void longPressStop1() {
  //Serial.println("Button 1 longPress stop");
}  // longPressStop1


// ... and the same for button 2:



// End
