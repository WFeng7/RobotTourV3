#ifndef Drivetrain_h
#define Drivetrain_h

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include <PIDController.h>
#include <Adafruit_BNO055.h>
#include <Definitions.h>
#include <HCSR04.h>

class Drivetrain {
    private: 
        AccelStepper y_stepper1, y_stepper2;
        int orientation = 0;
        int stepsPerRev = 200;
        int microstepMultiplier = 16;

        double velocityError = 0;
        double positionError = 0;

    public:
        PIDController pid = PIDController(TURN_KP, TURN_KI, TURN_KD, TURN_DT);
        double sharedYaw = 0.0;
        double yawOffset = 0;
        float frequency = 100.0;
        Adafruit_BNO055 bno;
        Drivetrain(AccelStepper* y_stepper1, AccelStepper* y_stepper2, Adafruit_BNO055* bno);
        void stepperSleep();
        void driveDistance(double dist, bool horizontal);
        void turn(int angle, double ROBOTDIAMETER);
        void stop();
        void driveTiles(double tiles, bool horizontal);
        void resetOrientation();
        int getOrientation();
        void setMicrostep(bool state);
        void turnRight();
        void turnLeft();
        void turnAround();
        void correctWithGyro(double angle, double ROBOTDIAMETER);

        void moveUntilSensor(double target, double temp);

        double getYaw();
        void updateYaw();
        void zeroYaw();

        double applyRollingAverage(double yaw);
};

#endif