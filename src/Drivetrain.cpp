#include <Arduino.h>
#include "Drivetrain.h"

Drivetrain::Drivetrain(AccelStepper* x1, AccelStepper* x2, AccelStepper* y1, AccelStepper* y2, MPU9250* mpu):
    x1(*x1),
    x2(*x2),
    y1(*y1),
    y2(*y2),
    mpu(*mpu) {}

void Drivetrain::set(double x1speed, double x2speed, double y1speed, double y2speed) { // steps per second
    x1.setSpeed(x1speed);
    x2.setSpeed(x2speed);
    y1.setSpeed(y1speed);
    y2.setSpeed(y2speed);
}

