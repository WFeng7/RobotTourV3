#ifndef Drivetrain_h
#define Drivetrain_h

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <MadgwickAHRS.h>
#include <math.h>
#include <PIDController.h>
#include <ContinuousStepper.h>
#include <Adafruit_BNO055.h>

class Drivetrain {
    private: 
        AccelStepper x_stepper1, x_stepper2, y_stepper1, y_stepper2;
        int orientation = 0;
        int stepsPerRev = 200;
        int microstepMultiplier = 16;

        double velocityError = 0;
        double positionError = 0;

    public:
        PIDController pid;
        volatile float sharedYaw = 0.0;
        double yawOffset = 0;
        float frequency = 100.0;
        Adafruit_BNO055 bno;
        Drivetrain(AccelStepper* x_stepper1, AccelStepper* x_stepper2, AccelStepper* y_stepper1, AccelStepper* y_stepper2, Adafruit_BNO055* bno);
        void stepperSleep();
        void driveDistance(double dist, bool horizontal);
        void turn(int angle, double ROBOTDIAMETER);
        void stop();
        void driveTiles(double tiles, bool horizontal);
        void resetOrientation();
        void setMicrostep(bool state);
        void turnRight();
        void turnLeft();
        void turnAround();
        void correctWithGyro(double angle, double ROBOTDIAMETER);
        bool atSetPoint();

        float getYaw();
        void updateYaw();
        void zeroYaw();
};

#endif