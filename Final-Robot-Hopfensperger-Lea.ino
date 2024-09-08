// ECE 314 Final Robot - Maze Solving
// Claire Hopfensperger & Teddy Lea

//-----------------------------------------------------------
// Include libraries, set up objects, define pins
//-----------------------------------------------------------

//// Necessary libraries for I2C LCD
//#include <Wire.h> // Library for I2C communication
//#include <LiquidCrystal_I2C.h> // Library for LCD
//LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

// Necessary defines/library for the IR receiver & remote
#define DECODE_NEC
#include <IRremote.hpp>

// Necessary servo stuff
#include <Servo.h>
Servo servo;

//-----------------------------------------------------------
// Pin/var delcarations
//-----------------------------------------------------------

// Wheel motors variables
const int RSPD = 175;        //Right Wheel PWM
const int LSPD = 185;        //Left Wheel PWM

const int LWhFwdPin = A0;  // IN3
const int LWhBwdPin = 7;  // IN4
const int LWhPWMPin = 6;  // ENB

const int RWhFwdPin = A1;  // IN1
const int RWhBwdPin = 4;  // IN2
const int RWhPWMPin = 5;  // ENA

// IR receiver/remote pins and vars
const int IR_Pin = 10;
int IRCode = 0;

// ISR vars
int gain = 12;
int delCntr = 0; 
volatile long cntrL, cntrR;
volatile long LIntTime, RIntTime;

// Robot movement state vars
int state = 0;
bool turn = false;

// Maze vars
double threshold;  // assuming inches
int degree;  // angle of robot

// Left Ultrasonic Module pins and vars
const int trigPinL = 13; // 10 microsecond high pulse causes chirp, wait 50 us
const int echoPinL = 12; // Width of high pulse indicates distance
long durationL;
double oldInchesL, inchesL, cmL;

// Right Ultrasonic Module pins and vars
const int trigPinR = 9; // 10 microsecond high pulse causes chirp, wait 50 us
const int echoPinR = 8; // Width of high pulse indicates distance
long durationR;
double oldInchesR, inchesR, cmR;

// Servo motor that aims ultrasonic sensor.
const int servoPin = 11; // PWM output for hobby servo
int pos;

//-----------------------------------------------------------
// Define statements (other)
//-----------------------------------------------------------

// IR button directions and codes
#define STOP 0x16   // Button 0
#define START 0xC   // Button 1

//-----------------------------------------------------------
// Setup
//-----------------------------------------------------------

void setup() 
{
  // Attach motor pins and set all to LOW
  pinMode(LWhFwdPin,OUTPUT);
  pinMode(LWhBwdPin,OUTPUT);
  pinMode(LWhPWMPin,OUTPUT);
  pinMode(RWhFwdPin,OUTPUT);
  pinMode(RWhBwdPin,OUTPUT);
  pinMode(RWhPWMPin,OUTPUT);

  digitalWrite(LWhFwdPin,LOW);
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(LWhPWMPin,LOW);
  
  digitalWrite(RWhFwdPin,LOW);
  digitalWrite(RWhBwdPin,LOW);
  digitalWrite(RWhPWMPin,LOW);

  // Attach interrupts to wheel sensors
  attachInterrupt(digitalPinToInterrupt(3), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), rightWhlCnt, CHANGE);

//  // Initialize L2C LCD
//  lcd.init();
//  lcd.backlight();
//  lcd.clear();
//  lcd.print("Ready to go!");

  // Set up IR pin
  pinMode(IR_Pin, INPUT);
  IrReceiver.begin(IR_Pin, ENABLE_LED_FEEDBACK); // Start the receiver

  // Initialize ISR vars
  cntrR = 0;
  cntrL = 0;
  LIntTime = 0;
  RIntTime = 0;

  // Maze setup stuff
  threshold = 6;  // inches
  degree = 0;

  // Configure input and output pins for left ultrasonic sensor
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  durationL = 0;
  oldInchesL = threshold;
  inchesL = threshold;
  cmL = 0;

  // Configure input and output pins for right ultrasonic sensor
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  durationR = 0;
  oldInchesR = threshold;
  inchesR = threshold;
  cmR = 0;

  // Configure servo
  servo.attach(servoPin);
  servoPos1();

  // Set up serial monitor
  Serial.begin(9600);
  Serial.println("Starting");

  // Delay time before robot starts moving forward
  delay(3000);
}

//-----------------------------------------------------------
// Helper functions
//-----------------------------------------------------------

/*
 * Return an int of the current decoded command detected
 * by the IR receiver.
 */
int getCommand() {
  if (IrReceiver.decode())
  {
    IRCode = IrReceiver.decodedIRData.command;  // Store the value of the remote press into num.
    IrReceiver.resume();                        // Enable receiving of the next value.
  }
  return IRCode;
}

/*
 * Set both wheels forward to move the robot straight.
 */
void goStraight() {
  // Left wheel settings
  digitalWrite(LWhFwdPin,HIGH);    // run left wheel forward
  digitalWrite(LWhBwdPin,LOW);

  // Right wheel settings
  digitalWrite(RWhFwdPin,HIGH);    // run right wheel forward
  digitalWrite(RWhBwdPin,LOW);
}

/*
 * Set both wheels backward to move the robot back.
 */
void goBack() {
  // Left wheel settings
  digitalWrite(LWhFwdPin,LOW);    // run left wheel backward
  digitalWrite(LWhBwdPin,HIGH);

  // Right wheel settings
  digitalWrite(RWhFwdPin,LOW);    // run right wheel backward
  digitalWrite(RWhBwdPin,HIGH);
}

/*
 * Set the right wheel forward and stop the left wheel
 * to move the robot left.
 */
void turnLeft() {
  // Left wheel settings
  digitalWrite(LWhFwdPin,LOW);    // stop left wheel
  digitalWrite(LWhBwdPin,LOW);

  // Right wheel settings
  digitalWrite(RWhFwdPin,HIGH);    // run right wheel forward
  digitalWrite(RWhBwdPin,LOW);
}

/*
 * Set the left wheel forward and stop the right wheel
 * to move the robot right.
 */
void turnRight() {
  // Left wheel settings
  digitalWrite(LWhFwdPin,HIGH);    // run left wheel forward
  digitalWrite(LWhBwdPin,LOW);

  // Right wheel settings
  digitalWrite(RWhFwdPin,LOW);    // stop right wheel
  digitalWrite(RWhBwdPin,LOW);
}

/*
 * Stop both wheels to stop the robot.
 */
void stopWheels() {
  // Left wheel settings
  digitalWrite(LWhFwdPin,LOW);    // stop left wheel
  digitalWrite(LWhBwdPin,LOW);

  // Right wheel settings
  digitalWrite(RWhFwdPin,LOW);    // stop right wheel
  digitalWrite(RWhBwdPin,LOW);
}

/*
 * Left wheel interrupt
 */
void leftWhlCnt()
{
  long intTime = micros();
  if(intTime > LIntTime + 1000L)
  {
    LIntTime = intTime;
    cntrL++;
  }
}

/*
 * Right wheel interrupt
 */
void rightWhlCnt()
{
  long intTime = micros();
  if(intTime > RIntTime + 1000L)
  {
    RIntTime = intTime;
    cntrR++;
  }
}

/*
 * Move robot at top speed forward for 1 second and backward for 1 second.
 */
void robotTest() {
  delay(1000);  // Wait 1 second to start
  
  // Go forward 1 second ---------
  // Left wheel forward
  digitalWrite(LWhFwdPin,HIGH);
  digitalWrite(LWhBwdPin,LOW);
  analogWrite(LWhPWMPin,255);

  // Right wheel forward
  digitalWrite(RWhFwdPin,HIGH);
  digitalWrite(RWhBwdPin,LOW);
  analogWrite(RWhPWMPin,255);

  delay(1000);

  // Go backward 1 second ---------
  // Left wheel back
  digitalWrite(LWhFwdPin,LOW);
  digitalWrite(LWhBwdPin,HIGH);
  analogWrite(LWhPWMPin,255);

  // Right wheel back
  digitalWrite(RWhFwdPin,LOW);
  digitalWrite(RWhBwdPin,HIGH);
  analogWrite(RWhPWMPin,255);

  delay(1000);

  // Stop robot ---------
  // Left wheel off
  digitalWrite(LWhFwdPin,LOW);
  digitalWrite(LWhBwdPin,LOW);
  analogWrite(LWhPWMPin,0);

  // Right wheel off
  digitalWrite(RWhFwdPin,LOW);
  digitalWrite(RWhBwdPin,LOW);
  analogWrite(RWhPWMPin,0);
}

/*
 * Front-facing ultrasonic sensor: read distance and convert
 * into inches and cm
 */
void readDistanceL() {
  // Read ultrasonic sensor distance
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);

  // Calculate distances with duration
  durationL = pulseIn(echoPinL, HIGH);
  inchesL = microsecondsToInches(durationL);
  cmL = microsecondsToCentimeters(durationL);
}

/*
 * Right-facing ultrasonic sensor: read distance and convert
 * into inches and cm
 */
void readDistanceR() {
  // Read ultrasonic sensor distance
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);

  // Calculate distances with duration
  durationR = pulseIn(echoPinR, HIGH);
  inchesR = microsecondsToInches(durationR);
  cmR = microsecondsToCentimeters(durationR);
}

/*
 * Converts ultrasonic sensor ms read to inches.
 */
double microsecondsToInches(long microseconds) {
  return (double) microseconds / 74 / 2;
}

/*
 * Converts ultrasonic sensor ms read to cm.
 */
double microsecondsToCentimeters(long microseconds) {
  return (double) microseconds / 29 / 2;
}

/*
 * Turn robot left 90 degrees
 */
void turnLeft90() { 
  long oldcntrL=cntrL;
  long oldcntrR=cntrR;
  cntrL=0;
  cntrR=0;
  
  while (cntrR < (cntrL + 42))
  { 
    analogWrite(RWhPWMPin,RSPD);
    analogWrite(LWhPWMPin,0);
  }
  
  cntrL=oldcntrL;
  cntrR=oldcntrR;

  // add 90 to "degree" to keep track of robot direction
  degree += 90;
}

/*
 * Turn robot right 90 degrees
 */
void turnRight90() { 
  long oldcntrL=cntrL;
  long oldcntrR=cntrR;
  cntrL=0;
  cntrR=0;
  
  while (cntrL < (cntrR + 43))
  { 
    analogWrite(LWhPWMPin,LSPD);
    analogWrite(RWhPWMPin,0);
  }
  
  cntrL=oldcntrL;
  cntrR=oldcntrR;

  // subtract 90 from "degree" to keep track of robot direction
  degree -= 90;
}

/*
 * Turn servo to position 1, 90 degrees
 */
void servoPos1() {
  servo.write(90);
  pos = 1;
}

/*
 * Turn servo to position 2, 0 degrees
 */
void servoPos2() {
  servo.write(0);
  pos = 2;
}

//-----------------------------------------------------------
// Main loop
//-----------------------------------------------------------

void loop()
{
  // Set vars to current distance readings
  oldInchesL = inchesL;
  oldInchesR = inchesR;

  // Get new ultrasonic distance readinds
  readDistanceL();
  readDistanceR();

  // Set motor PWMs both to "forward"
  goStraight();
  delay(100);
    
  // If goal detected, go to goal
  
  // If wall ahead, turn left
  if (inchesL <= threshold)
  {
    turnLeft90();

    // Pause robot for easier transition to straight
    stopWheels();
    delay(300);

    // Reset PWM values and set motors to "forward"
    analogWrite(RWhPWMPin,RSPD);
    analogWrite(LWhPWMPin,LSPD);
    goStraight();

    // Reset wheel encoder counts
    cntrL = 0;
    cntrR = 0;
  }

  // If corner right
  else if (oldInchesR + 2 < inchesR)
  {
    
    // go straight
    if (degree == 0)
    {
      // Set variables
      long tmpLcntr, tmpRcntr;
      tmpLcntr = cntrL;
      tmpRcntr = cntrR;
      delCntr = abs(tmpLcntr - tmpRcntr);
    
      // If left motor is faster than right motor, decrease left motor speed
      if(tmpLcntr > tmpRcntr)
      {
        analogWrite(RWhPWMPin,RSPD);
        analogWrite(LWhPWMPin, max(LSPD - int(gain * delCntr + .5), 0));
      }
  
      // If right motor is faster than left motor, decrease right motor speed
      else if(tmpLcntr < tmpRcntr)
      {
        analogWrite(RWhPWMPin, max(RSPD - int(gain * delCntr + .5), 0));
        analogWrite(LWhPWMPin,LSPD);
      }
  
      // If motors are same speed, keep speeds
      else
      {
        analogWrite(RWhPWMPin,RSPD);
        analogWrite(LWhPWMPin,LSPD);
      }
    }

    // turn right
    else  // (degree != 0)
    {
      // Delay soft right turn so that robot doesn't hit wall
      delay(430);
      turnRight90();

      // Pause robot for easier transition to straight
      stopWheels();
      delay(300);

      // Reset PWM values and set motors to "forward"
      analogWrite(RWhPWMPin,RSPD);
      analogWrite(LWhPWMPin,LSPD); 
      goStraight();
      delay(300);

      // Reset wheel encoder counts
      cntrL = 0;
      cntrR = 0;
    }
  }

  // If no reading on either, go straight
  else  
  {
    // Set variables
    long tmpLcntr, tmpRcntr;
    tmpLcntr = cntrL;
    tmpRcntr = cntrR;
    delCntr = abs(tmpLcntr - tmpRcntr);
  
    // If left motor is faster than right motor, decrease left motor speed
    if(tmpLcntr > tmpRcntr)
    {
      analogWrite(RWhPWMPin,RSPD);
      analogWrite(LWhPWMPin, max(LSPD - int(gain * delCntr + .5), 0));
    }

    // If right motor is faster than left motor, decrease right motor speed
    else if(tmpLcntr < tmpRcntr)
    {
      analogWrite(RWhPWMPin, max(RSPD - int(gain * delCntr + .5), 0));
      analogWrite(LWhPWMPin,LSPD);
    }

    // If motors are same speed, keep speeds
    else
    {
      analogWrite(RWhPWMPin,RSPD);
      analogWrite(LWhPWMPin,LSPD);
    }
  }
}
