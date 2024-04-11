#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
// #include <MPU9250.h>

#include "Drivetrain.h"
#include "Constants.h"

// Pins

#define X1_PIN1 1
#define X1_PIN2 2 
#define X1_PIN3 3
#define X1_PIN4 4

#define X2_PIN1 1
#define X2_PIN2 2
#define X2_PIN3 3
#define X2_PIN4 4

#define Y1_PIN2 1
#define Y1_PIN1 2
#define Y1_PIN3 3
#define Y1_PIN4 4

#define Y2_PIN1 1
#define Y2_PIN2 2
#define Y2_PIN3 3
#define Y2_PIN4 4

// Global Variables

AccelStepper x1, x2, y1, y2; // Stepper Motors
Drivetrain drivetrain;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000);
  
  x1 = AccelStepper(AccelStepper::FULL4WIRE, X1_PIN1, X1_PIN2, X1_PIN3, X1_PIN4, true);
  x2 = AccelStepper(AccelStepper::FULL4WIRE, X2_PIN1, X2_PIN2, X2_PIN3, X2_PIN4, true);
  y1 = AccelStepper(AccelStepper::FULL4WIRE, Y1_PIN1, Y1_PIN2, Y1_PIN3, Y1_PIN4, true);
  y2 = AccelStepper(AccelStepper::FULL4WIRE, Y2_PIN1, Y2_PIN2, Y2_PIN3, Y2_PIN4, true);

  // if (!mpu.setup(0x68)) {  // change to your own address
  //   while (1) {
  //     Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
  //     delay(5000);
  //   }
  // }

  // mpu.calibrateAccelGyro();
  // mpu.calibrateMag();

  drivetrain = Drivetrain(&x1, &x2, &y1, &y2);

}

void loop() {
  // put your main code here, to run repeatedly:
}