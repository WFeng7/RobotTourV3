#include <Arduino.h>
#include "Drivetrain.h"

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

#define WHEELDIAMETER 7.2
// #define ROBOTDIAMETER 20
#define TILE_LENGTH 50

#define MAXSPEED 1000
#define MAXACCELERATION 600

#define MAXTURNSPEED 875
#define MAXTURNACCELERATION 625

// #define NOMINAL_SPEED 800
// #define NOMINAL_ACCELERATION 400

#define NOMINAL_SPEED 1750
#define NOMINAL_ACCELERATION 1250

#define MICROSTEP 14

#define IMU_INTERRUPT 34

Drivetrain::Drivetrain(AccelStepper* x_stepper1, AccelStepper* x_stepper2, AccelStepper* y_stepper1, AccelStepper* y_stepper2):
    x_stepper1(*x_stepper1),
    x_stepper2(*x_stepper2),
    y_stepper1(*y_stepper1),
    y_stepper2(*y_stepper2) {
        pid.begin();
        pid.tune(1.0, 0.0, 0.0);
    }

void Drivetrain::stepperSleep() {
    digitalWrite(MICROSTEP, LOW);
    digitalWrite(SLEEP1, LOW);
    digitalWrite(SLEEP2, LOW);
}

void Drivetrain::createMutex() {
    yawMutex = xSemaphoreCreateMutex();
}

void Drivetrain::driveDistance(double pdist, bool horizontal) { // TO-DO: CONVERT DIST TO STEPS
    MultiStepper multi;
    int dist = (int)(pdist * stepsPerRev / (PI * WHEELDIAMETER));
    if(horizontal && (orientation % 180 == 0) || !horizontal && (orientation % 180 != 0)) {
        digitalWrite(SLEEP2, HIGH);
        // digitalWrite(SLEEP1, HIGH);
        // digitalWrite(SLEEP1, HIGH);
        // if (orientation == 180 || orientation == 270) {
        //     dist *= -1;
        // }
        x_stepper1.move(dist);
        if(dist > 0) {
            digitalWrite(X1_DIR, HIGH); 
            digitalWrite(X2_DIR, LOW);
        }
        else {
            digitalWrite(X1_DIR, LOW);
            digitalWrite(X2_DIR, HIGH);
        }
        // x_stepper2.move((orientation == 0 || orientation == 90 ? -1 : 1) * dist);
        x_stepper1.setMaxSpeed(NOMINAL_SPEED);
        // x_stepper2.setMaxSpeed(1000);
        // x_stepper1.setSpeed(speed);
        x_stepper1.setAcceleration(NOMINAL_ACCELERATION);
        // x_stepper1.setSpeed(speed);
        // x_stepper2.setSpeed(speed);
        // x_stepper2.setSpeed(speed);
        // multi.addStepper(x_stepper1);
        // multi.addStepper(x_stepper2);
        x_stepper1.runToPosition();
    }
    else {
        digitalWrite(SLEEP1, HIGH);
        // digitalWrite(SLEEP2, HIGH);
        // digitalWrite(SLEEP2, HIGH);
        // if (orientation == 180 || orientation == 270) {
        //     dist *= -1;
        // }
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
        y_stepper1.setMaxSpeed(NOMINAL_SPEED);
        // y_stepper2.setMaxSpeed(1000);
        // y_stepper1.setSpeed((orientation == 180 || orientation == 270 ? -1 : 1) * speed);
        // y_stepper2.setSpeed((orientation == 0 || orientation == 90 ? -1 : 1) * speed);
        y_stepper1.setAcceleration(NOMINAL_ACCELERATION);
        // multi.addStepper(y_stepper1);
        // multi.addStepper(y_stepper2);
        // multi.runSpeedToPosition();
        y_stepper1.runToPosition();
    }
    // delay(250);
    stepperSleep();
}

void Drivetrain::driveTiles(double tiles, bool horizontal) {
    driveDistance(tiles * TILE_LENGTH, horizontal);
}

void Drivetrain::updateYaw() {
    if (digitalRead(IMU_INTERRUPT) == LOW) {
        unsigned short fifoCnt = imu.fifoAvailable();

        if (fifoCnt > 0) {
            inv_error_t result = imu.dmpUpdateFifo();
            
            if (result == INV_SUCCESS) {
                imu.computeEulerAngles();
                imu.computeEulerAngles();

                float w = imu.calcQuat(imu.qw);
                float x = imu.calcQuat(imu.qx);
                float y = imu.calcQuat(imu.qy);
                float z = imu.calcQuat(imu.qz);

                // rotation.w = w;
                // rotation.x = x;
                // rotation.y = y;
                // rotation.z = z;

                // Euler angles = rotation.toEuler();
                // sharedYaw = angles.yaw;
                sharedYaw = imu.yaw;
            }
        }
    }
}

float Drivetrain::getYaw() {
    updateYaw();
    Serial.println(sharedYaw);
    return sharedYaw;
}

void Drivetrain::turn(int angle, double ROBOTDIAMETER) {

    // MultiStepper multi;
    float startYaw = getYaw();
    float targetYaw = startYaw + angle;

    double full_dist = (stepsPerRev * (ROBOTDIAMETER * PI * angle/360)/(PI * WHEELDIAMETER));
    int dist = (int)(stepsPerRev * (ROBOTDIAMETER * PI * angle/360)/(PI * WHEELDIAMETER)); // TO-DO: CONVERT DIST TO STEPS
    int microstep_dist = (int)((full_dist - dist) * microstepMultiplier);

    digitalWrite(SLEEP1, HIGH);
    // digitalWrite(SLEEP2, HIGH);
    if(full_dist > 0) {
        // digitalWrite(X1_DIR, HIGH);
        // digitalWrite(X2_DIR, HIGH);
        digitalWrite(Y1_DIR, HIGH);
        digitalWrite(Y2_DIR, HIGH);
    }
    else {
        // digitalWrite(X1_DIR, LOW);
        // digitalWrite(X2_DIR, LOW);
        digitalWrite(Y1_DIR, LOW);
        digitalWrite(Y2_DIR, LOW);
    }
    x_stepper1.setMaxSpeed(MAXTURNSPEED);
    y_stepper1.setMaxSpeed(MAXTURNSPEED);
    x_stepper1.setAcceleration(MAXTURNACCELERATION);
    y_stepper1.setAcceleration(MAXTURNACCELERATION);
    while (y_stepper1.distanceToGo() != 0) {
        // x_stepper1.run();
        y_stepper1.run();
    }
    y_stepper1.stop();

    pid.setpoint(targetYaw);
    pid.limit(-360.0, 360.0);
    setMicrostep(true);
    y_stepper1.setSpeed(MAXTURNSPEED);
    y_stepper1.setCurrentPosition(0);
    float yaw = getYaw();
    while (fabs(yaw - targetYaw) > 0.5) {
        yaw = getYaw();
        float output = pid.compute(yaw);
        float speed = (stepsPerRev * microstepMultiplier * (ROBOTDIAMETER * PI * output/360)/(PI * WHEELDIAMETER));
        y_stepper1.setSpeed(output);
        y_stepper1.runSpeed();
    }
    y_stepper1.stop();
    // setMicrostep(false);
    delay(250);
    stepperSleep();
}

void Drivetrain::turnRight() {
    turn(90, 19.8);
}

void Drivetrain::turnLeft() {
    turn(-90, 20);
}

void Drivetrain::turnAround() {
    turn(180, 19.7);
}

void Drivetrain::resetOrientation() {
    orientation = 0;
}

void Drivetrain::setMicrostep(bool state) {
    if (state) {
        digitalWrite(MICROSTEP, HIGH);
    }
    else {
        digitalWrite(MICROSTEP, LOW);
    }
}

void Drivetrain::stop() {
    x_stepper1.stop();
    x_stepper2.stop();
    y_stepper1.stop();
    y_stepper2.stop();

    x_stepper1.setCurrentPosition(0);
    x_stepper2.setCurrentPosition(0);
    y_stepper1.setCurrentPosition(0);
    y_stepper2.setCurrentPosition(0);

    digitalWrite(X1_DIR, LOW);
    digitalWrite(X2_DIR, LOW);
    digitalWrite(Y1_DIR, LOW);
    digitalWrite(Y2_DIR, LOW);
}