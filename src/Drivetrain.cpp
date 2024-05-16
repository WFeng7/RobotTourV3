#include <Arduino.h>
#include "Drivetrain.h"

#define X1_STEP 27 // motor 4
#define X1_DIR 18

#define X2_STEP 27 // motor 2
#define X2_DIR 15

#define Y1_STEP 4 // motor 1
#define Y1_DIR 32

#define Y2_STEP 4 // motor 3
#define Y2_DIR 25

#define motorInterfaceType 1

#define SLEEP1 19
#define SLEEP2 23

#define WHEELDIAMETER 7.2
// #define ROBOTDIAMETER 20
#define TILE_LENGTH 50

#define MAXSPEED 1000
#define MAXACCELERATION 600

#define MAXTURNSPEED 1500
#define MAXTURNACCELERATION 1250

// #define NOMINAL_SPEED 800
// #define NOMINAL_ACCELERATION 400

#define NOMINAL_SPEED 1500
#define NOMINAL_ACCELERATION 1250

#define MICROSTEP 14

Drivetrain::Drivetrain(AccelStepper* x_stepper1, AccelStepper* x_stepper2, AccelStepper* y_stepper1, AccelStepper* y_stepper2, Adafruit_BNO055* bno, ContinuousStepper<StepperDriver>* continous):
    x_stepper1(*x_stepper1),
    x_stepper2(*x_stepper2),
    y_stepper1(*y_stepper1),
    y_stepper2(*y_stepper2),
    bno(*bno),
    continuous(*continous) {
        pid.begin();
        pid.tune(90.0, 0.0, 0.0);
    }

void Drivetrain::stepperSleep() {
    digitalWrite(MICROSTEP, LOW);
    digitalWrite(SLEEP1, LOW);
    digitalWrite(SLEEP2, LOW);
}

void Drivetrain::driveDistance(double pdist, bool horizontal) { // TO-DO: CONVERT DIST TO STEPS
    int dist = (int)(pdist * stepsPerRev / (PI * WHEELDIAMETER));
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
    delay(250);
    // stepperSleep();
}

void Drivetrain::driveTiles(double tiles, bool horizontal) {
    driveDistance(tiles * TILE_LENGTH, horizontal);
}

void Drivetrain::updateYaw() {
    imu::Quaternion quat = bno.getQuat();

    double yy = quat.y() * quat.y();
    // double roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2*(quat.x() * quat.x() + yy));
    // double pitch = asin(2 * quat.w() * quat.y() - quat.x() * quat.z());
    double yaw = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2*(yy+quat.z() * quat.z()));

    // roll = roll * 180 / PI;
    // pitch = pitch * 180 / PI;
    yaw = yaw * 180 / PI;

    sharedYaw = yaw + yawOffset;
    if (sharedYaw > 180) {
        float prev = sharedYaw;
        sharedYaw -= 360;
        Serial.printf("%f > 180, Adjusted to: %f\n", prev, sharedYaw);
    }
    else if (sharedYaw < -180) {
        float prev = sharedYaw;
        sharedYaw += 360;
        Serial.printf("%f < 180, Adjusted to: %f\n", prev, sharedYaw);
    }
}

float Drivetrain::getYaw() {
    updateYaw();
    return -1*sharedYaw;
}

void Drivetrain::turn(int angle, double ROBOTDIAMETER) {
    // MultiStepper multi;
    double full_dist = (stepsPerRev * (ROBOTDIAMETER * PI * angle/360)/(PI * WHEELDIAMETER));
    int dist = (int)(stepsPerRev * (ROBOTDIAMETER * PI * angle/360)/(PI * WHEELDIAMETER)); // TO-DO: CONVERT DIST TO STEPS
    int microstep_dist = (int)((full_dist - dist) * microstepMultiplier);

    digitalWrite(SLEEP1, HIGH);
    digitalWrite(SLEEP2, HIGH);
    x_stepper1.move(dist);
    y_stepper1.move(dist);
    if(full_dist > 0) {
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
    x_stepper1.setMaxSpeed(MAXTURNSPEED);
    y_stepper1.setMaxSpeed(MAXTURNSPEED);
    x_stepper1.setAcceleration(MAXTURNACCELERATION);
    y_stepper1.setAcceleration(MAXTURNACCELERATION );
    Serial.println("FULL STEP");
    y_stepper1.runToPosition();
    Serial.println("MICROSTEP");
    setMicrostep(true);
    x_stepper1.move(microstep_dist);
    y_stepper1.move(microstep_dist);
    y_stepper1.runToPosition();
    setMicrostep(false);
    // delay(250);

    orientation += angle;
    if (orientation > 180) {
        orientation -= 360;
    }
    else if (orientation < -180) {
        orientation += 360;
    }
    // stepperSleep();
}

void Drivetrain::correctWithGyro(double angle, double ROBOTDIAMETER) {
    pid.setpoint(angle);
    pid.limit(-180.0, 180.0);
    // setMicrostep(true);
    float yaw = getYaw();
    long startTime = millis();
    // y_stepper1.move(100000);
    // y_stepper1.setCurrentPosition(0);
    while (fabs(yaw - angle) > 0.2 && millis() - startTime < 5000){
        double output = pid.compute(yaw);
        if(output > 0) {
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
        digitalWrite(SLEEP1, HIGH);
        digitalWrite(SLEEP2, HIGH);
        double targetStepsPerSecond = (output * stepsPerRev * (ROBOTDIAMETER * PI) / 360) / (PI * WHEELDIAMETER);
        y_stepper1.setSpeed(targetStepsPerSecond);
        y_stepper1.runSpeed();
        Serial.printf("Target: %f Current: %f\n", angle, yaw);
        yaw = getYaw();
        // Serial.println(continuous.isSpinning());
    }
    y_stepper1.setCurrentPosition(0);
    // y_stepper1.stop();
    // setMicrostep(false);
    delay(250);
    stepperSleep();
}

void Drivetrain::setYawOffset(double offset) {
    yawOffset = offset;
}

void Drivetrain::turnRight() {
    double startYaw = getYaw();
    turn(90, 18);
    double correctionAngle = orientation;
    if (correctionAngle > 180) {
        correctionAngle -= 360;
    }
    else if (correctionAngle < -180) {
        correctionAngle += 360;
    }
    Serial.println(correctionAngle);
    correctWithGyro(correctionAngle, 19);
}

void Drivetrain::turnLeft() {
    double startYaw = getYaw();
    turn(-90, 18);
    double correctionAngle = orientation;
    if (correctionAngle > 180) {
        correctionAngle -= 360;
    }
    else if (correctionAngle < -180) {
        correctionAngle += 360;
    }
    correctWithGyro(correctionAngle, 19);
}

void Drivetrain::turnAround() {
    double startYaw = getYaw();
    turn(180, 18);
    double correctionAngle = orientation;
    if (correctionAngle > 180) {
        correctionAngle -= 360;
    }
    else if (correctionAngle < -180) {
        correctionAngle += 360;
    }
    correctWithGyro(correctionAngle, 19);
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