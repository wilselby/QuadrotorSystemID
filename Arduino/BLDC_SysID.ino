/*
Wil Selby
Washington, DC
Aug 08, 2015

BLDC_SysID

This function records PWM inputs to a Brushless Direct Current (BLDC) motor
and records the propellor speed. The motor input is a PWM command and the propellor
speed is measured in rad/s. The propellor speed is measured using a photo interupter.
This sketch outputs a .csv file with timestamped PWM and rad/sec values for input
into a subsequent data anlaysis program such as MATLAB.

Ref - http://www.utopiamechanicus.com/article/arduino-analog-pins-photointerruptor-or-slotted-optical/
http://www.instructables.com/id/ESC-Programming-on-Arduino-Hobbyking-ESC/?ALLSTEPS
http://www.oreilly.de/catalog/arduinockbkger/Arduino_Kochbuch_englKap_18.pdf

 */

// Includes
#include <Servo.h>

// Variables
const int PI_input_pin = 2;  // digital pin input for reading the photo interupter
/*
const int numberOfEntries = 3;
const int results_size = 8;
volatile unsigned long microseconds;
volatile unsigned int index = 0;
volatile unsigned long results[numberOfEntries];
volatile unsigned long rpm = 0;
*/

volatile int postion = 0;
volatile boolean direction = false;
volatile double time = 0;
volatile double dTime = 0;
volatile double rpm;
const int spokes = 2;

Servo esc;

int value = 0;  //value to send to ESC
unsigned long timems = 0; //current time in milliseconds

// the setup routine runs once when you press reset:
void setup() {

  // Assign pin 9 to ESC
  esc.attach(9);  //31250 Hz

  // make the photo interupter's pin an input:
  pinMode(PI_input_pin, INPUT);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Attach interrupt to digital pin 2 (intterupt pin 0), run interrupt service routine on trigger
  attachInterrupt(0, analyze, FALLING);

  //results[0] = 0;
}

// the loop routine runs over and over again forever:
void loop() {

  timems = millis();

  // read the input pin:
  int PI_value = digitalRead(PI_input_pin);

  if (timems >= 0 and timems <= 2500)
  {
    value = 0;
  }

  if (timems > 2500 and timems <= 4500)
  {
    value = 1200;
  }

  if (timems > 4500 and timems <= 6500)
  {
    value = 1000;
  }

  if (timems > 6500 and timems <= 8500)
  {
    value = 1425;
  }

  if (timems > 8500 and timems <= 10500)
  {
    value = 1475;
  }

  if (timems > 10500 and timems <= 12500)
  {
    value = 1450;
  }

  if (timems > 12500 and timems <= 14500)
  {
    value = 1390;
  }

  if (timems > 14500 and timems <= 16500)
  {
    value = 1425;
  }

  if (timems > 16500 and timems <= 18500)
  {
    value = 1480;
  }

  if (timems > 18500 and timems <= 20500)
  {
    value = 1425;
  }

  if (timems > 20500 and timems <= 22500)
  {
    value = 1375;
  }

  if (timems > 22500)
  {
    value = 1000;
  }

  // Write PPM value to ESC
  esc.writeMicroseconds(value);

  // print out the data values to serial monitor
  Serial.print(timems);
  Serial.print(',');
  Serial.print(PI_value);
  Serial.print(',');
  Serial.print(value);
  Serial.print(',');
  Serial.print(dTime);
  Serial.print(',');
  Serial.print(rpm);
  Serial.println();

  delay(1);        // delay in between reads for stability
}

void analyze()
{

  dTime = micros()-time;
  time = time + dTime;
  rpm = 1/dTime * 1000000 * 60; // (1/spokes)

}



