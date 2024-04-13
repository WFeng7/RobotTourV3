#include <Arduino.h>
#include "Drivetrain.h"

#define SLEEP1 21
#define SLEEP2 23

#define WHEELDIAMETER 7.2
#define ROBOTDIAMETER 22.4

Drivetrain::Drivetrain(AccelStepper* x_stepper1, AccelStepper* x_stepper2, AccelStepper* y_stepper1, AccelStepper* y_stepper2):
    x_stepper1(*x_stepper1),
    x_stepper2(*x_stepper2),
    y_stepper1(*y_stepper1),
    y_stepper2(*y_stepper2) {}

void Drivetrain::stepperSleep() {
    digitalWrite(SLEEP1, LOW);
    digitalWrite(SLEEP2, LOW);
}

void Drivetrain::driveDistance(double pdist, double speed, bool horizontal) { // TO-DO: CONVERT DIST TO STEPS
    MultiStepper multi;
    int dist = (int)(pdist * 200 / (PI * WHEELDIAMETER));
    orientation = (orientation + 360) % 360;
    if(horizontal && (orientation % 180 == 0) || !horizontal && (orientation % 180 != 0)) {
        digitalWrite(SLEEP2, HIGH);
        x_stepper1.move((orientation == 180 || orientation == 270 ? -1 : 1) * dist);
        x_stepper2.move((orientation == 0 || orientation == 90 ? -1 : 1) * dist);
        x_stepper1.setMaxSpeed(1000);
        x_stepper2.setMaxSpeed(1000);
        x_stepper1.setSpeed((orientation == 180 || orientation == 270 ? -1 : 1) * speed);
        x_stepper2.setSpeed((orientation == 0 || orientation == 90 ? -1 : 1) * speed);
        multi.addStepper(x_stepper1);
        multi.addStepper(x_stepper2);
        multi.runSpeedToPosition();
    }
    else {
        digitalWrite(SLEEP1, HIGH);
        y_stepper1.move((orientation == 180 || orientation == 270 ? -1 : 1) * dist);
        y_stepper2.move((orientation == 0 || orientation == 90 ? -1 : 1) * dist);
        y_stepper1.setMaxSpeed(1000);
        y_stepper2.setMaxSpeed(1000);
        y_stepper1.setSpeed((orientation == 180 || orientation == 270 ? -1 : 1) * speed);
        y_stepper2.setSpeed((orientation == 0 || orientation == 90 ? -1 : 1) * speed);
        multi.addStepper(y_stepper1);
        multi.addStepper(y_stepper2);
        multi.runSpeedToPosition();
    }
    delay(250);
    stepperSleep();
}

void Drivetrain::turn(int angle, double speed) {
    MultiStepper multi;
    int dist = (int)(200 * (ROBOTDIAMETER * PI * angle/360)/(PI * WHEELDIAMETER)); // TO-DO: CONVERT DIST TO STEPS
    digitalWrite(SLEEP1, HIGH);
    digitalWrite(SLEEP2, HIGH);
    if(angle > 0) {
        x_stepper1.move(dist);
        x_stepper2.move(dist);
        y_stepper1.move(dist);
        y_stepper2.move(dist);
        x_stepper1.setMaxSpeed(1000);
        x_stepper2.setMaxSpeed(1000);
        y_stepper1.setMaxSpeed(1000);
        y_stepper2.setMaxSpeed(1000);
        x_stepper1.setSpeed(speed);
        x_stepper2.setSpeed(speed);
        y_stepper1.setSpeed(speed);
        y_stepper2.setSpeed(speed);
    }
    else {
        x_stepper1.move(-dist);
        x_stepper2.move(-dist);
        y_stepper1.move(-dist);
        y_stepper2.move(-dist);
        x_stepper1.setMaxSpeed(1000);
        x_stepper2.setMaxSpeed(1000);
        y_stepper1.setMaxSpeed(1000);
        y_stepper2.setMaxSpeed(1000);
        x_stepper1.setSpeed(-speed);
        x_stepper2.setSpeed(-speed);
        y_stepper1.setSpeed(-speed);
        y_stepper2.setSpeed(-speed);
    }
    multi.addStepper(x_stepper1);
    multi.addStepper(x_stepper2);
    multi.addStepper(y_stepper1);
    multi.addStepper(y_stepper2);
    multi.runSpeedToPosition();
    delay(250);
    stepperSleep();
    orientation = (orientation + (int)angle + 360) % 360;
}

void Drivetrain::stop() {
    x_stepper1.setCurrentPosition(0);
    x_stepper2.setCurrentPosition(0);
    y_stepper1.setCurrentPosition(0);
    y_stepper2.setCurrentPosition(0);
}