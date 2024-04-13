#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <string.h>
#include <iostream>
#include <fstream>
// #include <MPU9250.h>

#include "Drivetrain.h"
// #include "Constants.h"

// Pins

#define X1_STEP 19
#define X1_DIR 18

#define X2_STEP 4
#define X2_DIR 15

#define Y1_STEP 33
#define Y1_DIR 32

#define Y2_STEP 26
#define Y2_DIR 25

#define motorInterfaceType 1

#define SLEEP1 21
#define SLEEP2 23

#define BUTTON_PIN 13

// Global Variables

int t = 0;

int lastButtonState = HIGH;
int buttonState;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

AccelStepper x_stepper1 = AccelStepper(motorInterfaceType, X1_STEP, X1_DIR);
AccelStepper x_stepper2 = AccelStepper(motorInterfaceType, X2_STEP, X2_DIR);
AccelStepper y_stepper1 = AccelStepper(motorInterfaceType, Y1_STEP, Y1_DIR);
AccelStepper y_stepper2 = AccelStepper(motorInterfaceType, Y2_STEP, Y2_DIR);

Drivetrain drivetrain = Drivetrain(&x_stepper1, &x_stepper2, &y_stepper1, &y_stepper2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup");
  // if (!mpu.setup(0x68)) {  // change to your own address
  //   while (1) {
  //     Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
  //     delay(5000);
  //   }
  // }

  // mpu.calibrateAccelGyro();
  // mpu.calibrateMag();
  pinMode(SLEEP1, OUTPUT);
  pinMode(SLEEP2, OUTPUT);
  digitalWrite(SLEEP1, LOW);
  digitalWrite(SLEEP2, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // y_stepper1.setMaxSpeed(1000);
  // y_stepper1.setSpeed(300);
  // // digitalWrite(SLEEP1, HIGH);
  // y_stepper1.runSpeedToPosition();
  // // digitalWrite(SLEEP1, LOW);
  
}

void test() {
  drivetrain.driveDistance(175, 150, true);
  drivetrain.driveDistance(150, 150, false);

}

// void test() {
//   freopen("instructions.txt", "r", stdin);
//   std::string str;
//   while(!std::cin.eof()) {
//     char c; std::cin >> c; int val; std::cin >> val;
//     Serial.println(c + " " + val);
//     delay(500);
//     if(c == 'v') {
//       drivetrain.driveDistance(val * 25, 150, false);
//     }
//     else if(c == 'h') {
//       drivetrain.driveDistance(val * 25, 150, true);
//     }
//     else {
//       drivetrain.turn(val, 150);
//     }
//   }  
// }

void loop() {
  Serial.println("Loop");
  int reading = digitalRead(BUTTON_PIN);

  if(reading == LOW) {
    Serial.print("OK");
    delay(2000);
    test();
  }

}