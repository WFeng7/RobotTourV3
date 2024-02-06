#ifndef Drivetrain_h
#define Drivetrain_h

#include <AccelStepper.h>
#include <MPU9250.h>
#include "Constants.h"

class Drivetrain {
    private: 
        AccelStepper x1, x2, y1, y2;
        MPU9250 mpu;
        bool rotated;
    public:
        Drivetrain(AccelStepper* x1, AccelStepper* x2, AccelStepper* y1, AccelStepper* y2, MPU9250* mpu);
        void set(double x1speed, double x2speed, double y1speed, double y2speed);
        void driveDistance(double distance);
        void update();
        void drive(double speed, bool horizontal);
        void turnSpeed(double speed);
        void turn(double angle);
        void stop();

};

#endif