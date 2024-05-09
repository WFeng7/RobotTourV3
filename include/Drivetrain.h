#ifndef Drivetrain_h
#define Drivetrain_h

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <SparkFunMPU9250-DMP.h>
#include <MadgwickAHRS.h>
#include <math.h>

struct Euler {
    float roll, pitch, yaw;
};

struct Quaternion {
    float w, x, y, z;

    Euler toEuler() {
        Euler angles;
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        angles.roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = std::sqrt(1 + 2 * (w * y - x * z));
        double cosp = std::sqrt(1 - 2 * (w * y - x * z));
        angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        angles.yaw = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }
};

class Drivetrain {
    private: 
        AccelStepper x_stepper1, x_stepper2, y_stepper1, y_stepper2;
        int orientation = 0;
        int stepsPerRev = 200;
        int microstepMultiplier = 16;
        Quaternion rotation;
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