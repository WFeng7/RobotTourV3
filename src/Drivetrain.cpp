#include <Arduino.h>
#include "Drivetrain.h"

#define X1_STEP 4
#define X1_DIR 18

#define X2_STEP 4
#define X2_DIR 15

#define Y1_STEP 4
#define Y1_DIR 32

#define Y2_STEP 4
#define Y2_DIR 25

#define motorInterfaceType 1

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

void Drivetrain::driveDistance(double pdist, bool horizontal) { // TO-DO: CONVERT DIST TO STEPS
    MultiStepper multi;
    int dist = (int)(pdist * 200 / (PI * WHEELDIAMETER));
    if(horizontal && (orientation % 180 == 0) || !horizontal && (orientation % 180 != 0)) {
        digitalWrite(SLEEP2, HIGH);
        x_stepper1.move(dist);
        // x_stepper2.move((orientation == 0 || orientation == 90 ? -1 : 1) * dist);
        x_stepper1.setMaxSpeed(1000);
        // x_stepper2.setMaxSpeed(1000);
        // x_stepper1.setSpeed(speed);
        x_stepper1.setAcceleration(600);
        // x_stepper1.setSpeed(speed);
        // x_stepper2.setSpeed(speed);
        // x_stepper2.setSpeed(speed);
        // multi.addStepper(x_stepper1);
        // multi.addStepper(x_stepper2);
        while (x_stepper1.run()) {
            if(dist > 0) {
                digitalWrite(X1_DIR, HIGH); 
                digitalWrite(X2_DIR, LOW);
            }
            else {
                digitalWrite(X1_DIR, LOW);
                digitalWrite(X2_DIR, HIGH);
            }
        }
    }
    else {
        digitalWrite(SLEEP1, HIGH);
        y_stepper1.move(dist);
        // y_stepper1.move((orientation == 180 || orientation == 270 ? -1 : 1) * dist);
        // y_stepper2.move((orientation == 0 || orientation == 90 ? -1 : 1) * dist);
        if(dist > 0) {
            digitalWrite(Y1_DIR, HIGH); // clockwise
            digitalWrite(Y2_DIR, LOW);
        }
        else {
            digitalWrite(Y1_DIR, LOW);
            digitalWrite(Y2_DIR, HIGH);
        }
        y_stepper1.setMaxSpeed(1000);
        // y_stepper2.setMaxSpeed(1000);
        // y_stepper1.setSpeed((orientation == 180 || orientation == 270 ? -1 : 1) * speed);
        // y_stepper2.setSpeed((orientation == 0 || orientation == 90 ? -1 : 1) * speed);
        y_stepper1.setAcceleration(600);
        // multi.addStepper(y_stepper1);
        // multi.addStepper(y_stepper2);
        // multi.runSpeedToPosition();
        y_stepper1.runToPosition();
    }
    delay(500);
    stepperSleep();
}

void Drivetrain::turn(int angle) {
    // MultiStepper multi;
    int dist = (int)(200 * (ROBOTDIAMETER * PI * angle/360)/(PI * WHEELDIAMETER)); // TO-DO: CONVERT DIST TO STEPS
    digitalWrite(SLEEP1, HIGH);
    digitalWrite(SLEEP2, HIGH);
    x_stepper1.move(dist);
    if(dist > 0) {
        digitalWrite(X1_DIR, HIGH);
        digitalWrite(X2_DIR, HIGH);
        digitalWrite(Y1_DIR, HIGH);
        digitalWrite(Y2_DIR, HIGH);
    }
    else {
        digitalWrite(X1_DIR, LOW);
        digitalWrite(X2_DIR, LOW);
        digitalWrite(Y1_DIR, LOW);
        digitalWrite(Y2_DIR, LOW);
    }
    x_stepper1.setMaxSpeed(500);
    x_stepper1.setAcceleration(300);
    x_stepper1.runToPosition();
    delay(500);
    stepperSleep();
    orientation = (orientation + (int)angle + 360) % 360;
}

void Drivetrain::stop() {
    x_stepper1.setCurrentPosition(0);
    x_stepper2.setCurrentPosition(0);
    y_stepper1.setCurrentPosition(0);
    y_stepper2.setCurrentPosition(0);

    digitalWrite(X1_DIR, LOW);
    digitalWrite(X2_DIR, LOW);
    digitalWrite(Y1_DIR, LOW);
    digitalWrite(Y2_DIR, LOW);
}