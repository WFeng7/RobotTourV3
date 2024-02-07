#ifndef Drivetrain_h
#define Drivetrain_h

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <MPU9250.h>
#include "Constants.h"

class Drivetrain {
    private: 
        AccelStepper x1, x2, y1, y2;
        MPU9250 mpu;
        int orientation = 0;
    public:
        Drivetrain(AccelStepper* x1, AccelStepper* x2, AccelStepper* y1, AccelStepper* y2, MPU9250* mpu);
        void set(double speed);
        void driveDistance(double dist, double speed, bool horizontal);
        void turn(int angle, double speed);
        void stop();

};

#endif