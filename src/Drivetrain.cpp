#include <Arduino.h>
#include "Drivetrain.h"

Drivetrain::Drivetrain(AccelStepper* x1, AccelStepper* x2, AccelStepper* y1, AccelStepper* y2, MPU9250* mpu):
    x1(*x1),
    x2(*x2),
    y1(*y1),
    y2(*y2),
    mpu(*mpu) {}

void Drivetrain::set(double speed) { // steps per second
    x1.setSpeed(speed);
    x2.setSpeed(speed);
    y1.setSpeed(speed);
    y2.setSpeed(speed);
}

void Drivetrain::driveDistance(double dist, double speed, bool horizontal) { 
    MultiStepper multi;
    orientation = (orientation + 360) % 360;
    if(horizontal && (orientation % 180 == 0) || !horizontal && (orientation % 180 != 0)) {
        x1.move((orientation == 180 || orientation == 270 ? -1 : 1) * dist);
        x2.move((orientation == 0 || orientation == 90 ? -1 : 1) * dist);
        x1.setSpeed((orientation == 180 || orientation == 270 ? -1 : 1) * speed);
        x2.setSpeed((orientation == 0 || orientation == 90 ? -1 : 1) * speed);
        multi.addStepper(x1);
        multi.addStepper(x2);
        multi.runSpeedToPosition();
    }
    else {
        y1.move((orientation == 180 || orientation == 270 ? -1 : 1) * dist);
        y2.move((orientation == 0 || orientation == 90 ? -1 : 1) * dist);
        y1.setSpeed((orientation == 180 || orientation == 270 ? -1 : 1) * speed);
        y2.setSpeed((orientation == 0 || orientation == 90 ? -1 : 1) * speed);
        multi.addStepper(y1);
        multi.addStepper(y2);
        multi.runSpeedToPosition();
    }
}

void Drivetrain::turn(int angle, double speed) {
    MultiStepper multi;
    double dist = (angle * 2 * PI) / 360 * Constants::ROBOT_RADIUS/4;
    x1.move(dist);
    x2.move(dist);
    y1.move(dist);
    y2.move(dist);
    if(angle > 0) {
        x1.setSpeed(speed);
        x2.setSpeed(speed);
        y1.setSpeed(speed);
        y2.setSpeed(speed);
    }
    else {
        x1.setSpeed(-speed);
        x2.setSpeed(-speed);
        y1.setSpeed(-speed);
        y2.setSpeed(-speed);
    }
    multi.addStepper(x1);
    multi.addStepper(x2);
    multi.addStepper(y1);
    multi.addStepper(y2);
    multi.runSpeedToPosition();
    orientation = (orientation + (int)angle + 360) % 360;
}

void Drivetrain::stop() {
    x1.setCurrentPosition(0);
    x2.setCurrentPosition(0);
    y1.setCurrentPosition(0);
    y2.setCurrentPosition(0);
}