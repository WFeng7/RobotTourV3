#include <Arduino.h>
#include "Drivetrain.h"

#include <Definitions.h>

Drivetrain::Drivetrain(AccelStepper* x_stepper1, AccelStepper* x_stepper2, AccelStepper* y_stepper1, AccelStepper* y_stepper2, Adafruit_BNO055* bno):
    x_stepper1(*x_stepper1),
    x_stepper2(*x_stepper2),
    y_stepper1(*y_stepper1),
    y_stepper2(*y_stepper2),
    bno(*bno) {
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
    y_stepper1.move(dist);
    if(dist > 0) {
        digitalWrite(Y1_DIR, HIGH); // clockwise
        digitalWrite(Y2_DIR, LOW);
    }
    else {
        digitalWrite(Y1_DIR, LOW);
        digitalWrite(Y2_DIR, HIGH);
    }
    y_stepper1.setMaxSpeed(NOMINAL_SPEED);
    y_stepper1.setAcceleration(NOMINAL_ACCELERATION);
    y_stepper1.runToPosition();
    delay(250);
}

void Drivetrain::driveTiles(double tiles, bool horizontal) {
    driveDistance(tiles * TILE_LENGTH, horizontal);
}

void Drivetrain::updateYaw() {
    imu::Quaternion quat = bno.getQuat();

    double yy = quat.y() * quat.y();
    double yaw = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2*(yy+quat.z() * quat.z()));

    yaw = yaw * 180 / PI;

    sharedYaw = yaw + yawOffset;
    if (sharedYaw > 180) {
        float prev = sharedYaw;
        sharedYaw -= 360;
        PRINTER.printf("%f > 180, Adjusted to: %f\n", prev, sharedYaw);
    }
    else if (sharedYaw < -180) {
        float prev = sharedYaw;
        sharedYaw += 360;
        PRINTER.printf("%f < 180, Adjusted to: %f\n", prev, sharedYaw);
    }
}

float Drivetrain::getYaw() {
    updateYaw();
    return -1*sharedYaw;
}

void Drivetrain::turn(int angle, double ROBOTDIAMETER) {
    double full_dist = (stepsPerRev * (ROBOTDIAMETER * PI * angle/360)/(PI * WHEELDIAMETER));
    int dist = (int)(stepsPerRev * (ROBOTDIAMETER * PI * angle/360)/(PI * WHEELDIAMETER));
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

    orientation += angle;
    if (orientation > 180) {
        orientation -= 360;
    }
    else if (orientation < -180) {
        orientation += 360;
    }
}

bool Drivetrain::atSetPoint() {
    positionError = pid.error;
    velocityError = pid.dErr;

    if (fabs(positionError) < 0.1 && fabs(velocityError) < 0.1) {
        return true;
    }
    return false;
}

void Drivetrain::correctWithGyro(double angle, double ROBOTDIAMETER) {
    pid.setpoint(angle);
    pid.limit(-180.0, 180.0);
    // setMicrostep(true);
    float yaw = getYaw();
    long startTime = millis();
    long onTargetStartTime = millis();
    // y_stepper1.move(100000);
    // y_stepper1.setCurrentPosition(0);
    while (millis() - startTime < 5000){
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
        // PRINTER.printf("Position Error: %f Velocity Error: %f\n", positionError, velocityError);
        // PRINTER.printf("Target: %f Current: %f\n", angle, yaw);
        yaw = getYaw();

        if (atSetPoint()) {
            // PRINTER.printf("Position Error: %f Velocity Error: %f\n", positionError, velocityError);
            break;
        }
    }
    y_stepper1.setCurrentPosition(0);
    delay(250);

    stepperSleep();
}

void Drivetrain::zeroYaw() {
    yawOffset = 0.0;
    float yaw = getYaw();
    yawOffset = yaw;
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