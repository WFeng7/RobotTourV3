#ifndef Drivetrain_h
#define Drivetrain_h

#include <AccelStepper.h>
#include <MultiStepper.h>
// #include <MPU9250.h>
// #include "Constants.h"

class Drivetrain {
    private: 
        AccelStepper x_stepper1, x_stepper2, y_stepper1, y_stepper2;
        int orientation = 0;
    public:
        Drivetrain(AccelStepper* x_stepper1, AccelStepper* x_stepper2, AccelStepper* y_stepper1, AccelStepper* y_stepper2);
        void stepperSleep();
        // void set(double speed);
        void driveDistance(double dist, double speed, bool horizontal);
        void turn(int angle, double speed);
        void stop();

};

#endif