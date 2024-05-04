#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
// #include <MPU9250.h>

#include "Drivetrain.h"
// #include "Constants.h"

// Pins

#define X1_STEP 19 // motor 4
#define X1_DIR 18

#define X2_STEP 19 // motor 2
#define X2_DIR 15

#define Y1_STEP 4 // motor 1
#define Y1_DIR 32

#define Y2_STEP 4 // motor 3
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

AccelStepper x_stepper1 = AccelStepper(motorInterfaceType, X1_STEP, 2);
AccelStepper x_stepper2 = AccelStepper(motorInterfaceType, X2_STEP, 2);
AccelStepper y_stepper1 = AccelStepper(motorInterfaceType, Y1_STEP, 2);
AccelStepper y_stepper2 = AccelStepper(motorInterfaceType, Y2_STEP, 2);

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

  pinMode(X1_STEP, OUTPUT);
  pinMode(X1_DIR, OUTPUT);
  pinMode(X2_STEP, OUTPUT);
  pinMode(X2_DIR, OUTPUT);
  pinMode(Y1_STEP, OUTPUT);
  pinMode(Y1_DIR, OUTPUT);
  pinMode(Y2_STEP, OUTPUT);
  pinMode(Y2_DIR, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // y_stepper1.setMaxSpeed(1000);
  // y_stepper1.setSpeed(300);
  // // digitalWrite(SLEEP1, HIGH);
  // y_stepper1.runSpeedToPosition();
  // // digitalWrite(SLEEP1, LOW);
  
}

std::vector<std::string> split(const std::string& s, char c) {
  std::vector<std::string> result;
  size_t begin = 0;
  while (true) {
    size_t end = s.find_first_of(c, begin);
    result.push_back(s.substr(begin, end - begin));

    if (end == std::string::npos) {
      break;
    }

    begin = end + 1;
  }
  return result;
}

void test() {
  Serial.println("Enter command: ");
  String command = Serial.readStringUntil('\n');

  while (command == "") {
    command = Serial.readStringUntil('\n');
  }

  Serial.println("Executing: " + command);

  std::vector<std::string> commands = split(command.c_str(), ' ');
  // if (commands[0] == "x") {
  //   drivetrain.driveTiles(std::stoi(commands[1]), false);
  // } else if (commands[0] == "y") {
  //   drivetrain.driveTiles(std::stoi(commands[1]), true);
  // } else if (commands[0] == "turn") {
  //   drivetrain.turn(std::stoi(commands[1]));
  // } else if (commands[0] == "xd") {
  //   drivetrain.driveDistance(std::stoi(commands[1]), false);
  // } else if (commands[0] == "yd") {
  //   drivetrain.driveDistance(std::stoi(commands[1]), true);
  // }
  // else {
  //   Serial.println("Invalid command");
  // }

  if (commands[0] == "x1" || commands[0] == "x2") {
    x_stepper1.move(500);
    x_stepper1.setMaxSpeed(1000);
    x_stepper1.setAcceleration(600);
    x_stepper1.runToPosition();
  }
  else if (commands[0] == "y1" || commands[0] == "y2") {
    y_stepper1.move(500);
    y_stepper1.setMaxSpeed(1000);
    y_stepper1.setAcceleration(600);
    y_stepper1.runToPosition();
  }
  else {
    Serial.println("Invalid command");
  }
}

void run() {
  // drivetrain.driveTiles(3, false);
  // drivetrain.driveTiles(3, true);
  // drivetrain.turn(180);
  drivetrain.turn(90);
  drivetrain.turn(-90);
}

void loop() {
  Serial.println("Loop");
  int reading = digitalRead(BUTTON_PIN);

  if(reading == LOW) {
    drivetrain.resetOrientation();
    Serial.print("OK");
    delay(2000);
    run();
  }
  // test();
}