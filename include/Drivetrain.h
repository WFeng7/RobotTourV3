#ifndef Drivetrain_h
#define Drivetrain_h

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <SparkFunMPU9250-DMP.h>
#include <MadgwickAHRS.h>
#include <math.h>

class Drivetrain {
    private: 
        AccelStepper x_stepper1, x_stepper2, y_stepper1, y_stepper2;
        int orientation = 0;
        int stepsPerRev = 200;
        int microstepMultiplier = 16;
    public:
        volatile float sharedYaw = 0.0;
        MPU9250_DMP imu;
        float frequency = 100.0;
        SemaphoreHandle_t yawMutex;
        Madgwick filter;
        TaskHandle_t sensorTaskHandle = NULL;
        Drivetrain(AccelStepper* x_stepper1, AccelStepper* x_stepper2, AccelStepper* y_stepper1, AccelStepper* y_stepper2);
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

        float getYaw();
        void updateYaw();
        void createMutex();
};

#endif