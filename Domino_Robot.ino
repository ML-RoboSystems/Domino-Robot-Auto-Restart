/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (Domino Robot Code)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/
#include "SparkFun_TB6612.h"                                //motor driver library
#include <Servo.h>                                          //servo controller library
#include <OneButton.h>                                      //button handling library
#include <avr/wdt.h>                                        //watchdog library for rebooting

#define KP                0.4                               //P term for PID controller
#define KD                0.8                               //D term for PID controller
#define SERVO_RIGHT       20                                //PWM value to make the servo move right
#define SERVO_LEFT        180                               //PWM value to make the servo move left.
#define SERVO_PIN         13                                //PWM pin for domino dispensing servo
#define AIN1              9                                 //motor driver
#define BIN1              7                                 //motor driver
#define AIN2              10                                //motor driver
#define BIN2              6                                 //motor driver
#define PWMA              11                                //motor driver
#define PWMB              5                                 //motor driver
#define STBY              8                                 //motor driver
#define OFFSET_A          1                                 //Switch to -1 if the motor is running backwards
#define OFFSET_B          1                                 //Switch to -1 if the motor is running backwards
#define BUTTON_PIN        12                                //The pin the switch for detecting domino outage is connected to.
#define LEFT_SENSOR_PIN   A1                                //Left line sensor pin
#define RIGHT_SENSOR_PIN  A0                                //Right sensor pin
#define TOP_SPEED         90                                //PWM value for max speed of motors
#define DISPENSE_DISTANCE 500000                            //Value combinedMotorSpeed needs to reach to drop a domino. Decreasing this puts dominoes closer together.

//set up the servo                          
Servo servoMotor;                                           //Create an instance of a servo motor object

//set up the drive motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, OFFSET_A, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);
int combinedmotorspeed, leftMotorSpeed, rightMotorSpeed;    //creating variables like this initializes all their values to 0.

// Define variables for PID control
float lastError, derivative, controlSignal;                 //these initialize to 0/
int error;

//Line sensor variables
int leftSensorValue, rightSensorValue;                      //variables for storing the value of the line sensors

//Out of dominoes switch setup
OneButton stopButton = OneButton(BUTTON_PIN, false, false); //Active high, no internal pullup resistor
bool stopButtonPressed = false;                             //Boolean for tracking the state of the button

//Distance based domino dropping variables
unsigned long distanceSinceLastDrop = DISPENSE_DISTANCE;    //Starting with this as DISPENSE_DISTANCE makes the vehicle drop a domino immediately.
bool dominoDropped = false;

void setup() {
  digitalWrite(STBY, HIGH);
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(SERVO_LEFT);
  
  stopButton.setPressMs(200);
  stopButton.attachLongPressStart([]() {
    stopButtonPressed = true;  // Button is pressed and held
  });
  stopButton.attachLongPressStop([]() {
    stopButtonPressed = false; // Button is released
  });
  delay(800);
}

void reboot() {
  wdt_enable(WDTO_15MS); // Enable watchdog with 15ms timeout
  while (1); // Wait for the watchdog to reset the Arduino
}

void loop() {
  //First, take care of the line following functions. We want to follow the line regardless of whether or not we're out of dominoes.
  //Even when the vehicle runs out of dominoes, it should follow the line to the next point where a domino needs to be, then stop.
  
  //Read the line sensors and calculate control signal.
  leftSensorValue = analogRead(LEFT_SENSOR_PIN);                //Read line sensor values
  rightSensorValue = analogRead(RIGHT_SENSOR_PIN);
  error = leftSensorValue - rightSensorValue;                   //Calculate the error (difference between the two sensor values)
  derivative = error - lastError;                               //Calculate the derivative term, which is how fast the error is changing
  controlSignal = (KP * error) + (KD * derivative);             //Calculate the control signal
  lastError = error;                                            //Store the current error for the next iteration

  // Calculate motor speeds based on the control signal
  leftMotorSpeed = TOP_SPEED - controlSignal;                   //If the error is positive, this slows down the motor on the left
  rightMotorSpeed = TOP_SPEED + controlSignal;                  //and speeds up the motor on the right. Vice versa if the error is negative.

  // Ensure motor speeds are within the valid range
  leftMotorSpeed = constrain(leftMotorSpeed, 0, TOP_SPEED);     //This makes the speed be somewhere between 0 and TOP_SPEED
  rightMotorSpeed = constrain(rightMotorSpeed, 0, TOP_SPEED);
  
  // Apply motor speeds
  motor1.drive(leftMotorSpeed);
  motor2.drive(rightMotorSpeed);
  combinedmotorspeed = leftMotorSpeed + rightMotorSpeed;
  distanceSinceLastDrop += combinedmotorspeed;

  //That's it for the line following functions, now move on to dispensing dominoes at the appropriate time.

  stopButton.tick();                    //Check the switch to see if we're out of dominoes or not. This has to be called every time through the loop()
  
  if (!stopButtonPressed) {             //If the limit switch is not pressed, this is true and the domino dispensing code executes
    //figure out when to dispense a domino based on how far the robot has travelled.

    if ((distanceSinceLastDrop >= DISPENSE_DISTANCE) && !dominoDropped) {              //if we've travelled far enough and a domino hasn't been dropped yet
      servoMotor.write(SERVO_RIGHT);
      distanceSinceLastDrop = 0;                                                       //reset the distance count to 0 because we just dropped a domino
      dominoDropped = true;                                                            //domino has been dropped
    } else if ((distanceSinceLastDrop >= 0.7 * DISPENSE_DISTANCE) && dominoDropped) {  //if a domino was dropped and we're far enough away to close the domino gate without jamming
      servoMotor.write(SERVO_LEFT);                                                    //pick up a new domino for dropping
      dominoDropped = false;                                                           //we haven't dropped the new domino, so now this is false
    }
  }
  else { // This code executes if the limit switch is pressed (vehicle is out of dominoes)
  if (!dominoDropped) { // Check if a domino has been dropped
    servoMotor.write(SERVO_RIGHT); // If a domino is still in the magazine, drop the domino
    distanceSinceLastDrop = 0; // Reset the distanceSinceLastDrop
    dominoDropped = true;
  }
  
  // Stop motors immediately when out of dominoes
  motor1.brake();
  motor2.brake();
  
  // Wait for button release with non-blocking check
  while (true) {
    stopButton.tick(); // Critical - update button state
    if (!stopButtonPressed) {
      delay(15000);
      reboot(); //Reboot when button released
    }
    delay(10); // Small delay to prevent watchdog reset
  }
}
}