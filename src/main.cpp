#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ContinuousStepper.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>

#include "Drivetrain.h"
#include <math.h>
#include <Adafruit_BNO055.h>
#include <Pathfinding.h>

#include <Definitions.h>

// Global Variables

int t = 0;

const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

adafruit_bno055_offsets_t bno_offsets;

int lastButtonState = HIGH;
int buttonState;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

AccelStepper x_stepper1 = AccelStepper(motorInterfaceType, X1_STEP, 2);
AccelStepper x_stepper2 = AccelStepper(motorInterfaceType, X2_STEP, 2);
AccelStepper y_stepper1 = AccelStepper(motorInterfaceType, Y1_STEP, 2);
AccelStepper y_stepper2 = AccelStepper(motorInterfaceType, Y2_STEP, 2);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Pathfinding pathfinding = Pathfinding();

std::vector<std::string> path;

Drivetrain drivetrain = Drivetrain(&x_stepper1, &x_stepper2, &y_stepper1, &y_stepper2, &bno);

void setup() {
  digitalWrite(2, LOW);
  Serial.begin(115200);

  Wire.begin();
  delay(2000);
  Serial.println("Setup");

  pinMode(SLEEP1, OUTPUT);
  pinMode(SLEEP2, OUTPUT);
  pinMode(MICROSTEP, OUTPUT);
  digitalWrite(SLEEP1, LOW);
  digitalWrite(SLEEP2, LOW);
  digitalWrite(MICROSTEP, LOW);

  pinMode(X1_STEP, OUTPUT);
  pinMode(X1_DIR, OUTPUT);
  pinMode(X2_STEP, OUTPUT);
  pinMode(X2_DIR, OUTPUT);
  pinMode(Y1_STEP, OUTPUT);
  pinMode(Y1_DIR, OUTPUT);
  pinMode(Y2_STEP, OUTPUT);
  pinMode(Y2_DIR, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("IMU Setup");
  if (!drivetrain.bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  drivetrain.bno.enterNormalMode();
  drivetrain.bno.setExtCrystalUse(true);
  drivetrain.bno.enableAutoRange(true);

  // CHANGE THIS:
  pathfinding.setStart(0);
  pathfinding.setTarget({1, 3});
  pathfinding.addGate(0, 1);
  pathfinding.addGate(2, 0);
  pathfinding.addGate(3, 0);
  pathfinding.addGate(3, 3);
  pathfinding.addWood(0, 0, 0, 1);
  pathfinding.addWood(0, 1, 1, 1);
  pathfinding.addWood(1, 3, 1, 2);
  pathfinding.addWood(1, 3, 2, 3);
  pathfinding.addWood(1, 0, 2, 0);
  pathfinding.addWood(2, 0, 3, 0);
  pathfinding.addWood(2, 1, 3, 1);
  pathfinding.addWood(3, 2, 3, 3);

  pathfinding.findPath();

  path = pathfinding.getPath();

  delay(5000);
}

void run() {
  for(std::string &s : path) {
    if(s[0] == 't') {
      // drivetrain.driveDistance(std::stoi(s.substr(1)), false);
      Serial.println("Drive distance");
    }
    else if(s[0] == 'r') {
      drivetrain.turnRight();
    }
    else if(s[0] == 'l') {
      drivetrain.turnLeft();
    }
    else if(s[0] == 'a') {
      drivetrain.turnAround();
    }
  }
}

void loop() {
  digitalWrite(2, HIGH);
  int reading = digitalRead(BUTTON_PIN);

  if(reading == LOW) {
    drivetrain.resetOrientation();
    Serial.println(drivetrain.getYaw());
    while (fabs(drivetrain.getYaw()) > 0.5) {
      double yaw = drivetrain.getYaw();
      drivetrain.setYawOffset(yaw);
      Serial.println(drivetrain.getYaw());
    }
    Serial.print("OK");
    delay(2000);
    run();
  }
}