// #define USING_TELNET 1

// #define SAVE_CALIBRATION

#ifdef USING_TELNET
  #include <Arduino.h>
  #include <Wifi.h>
  #include <TelnetStream.h>
  #include <arduino_secrets.h>
  #include <TimeLib.h>
#endif

#define Y1_STEP 4 // motor 1
#define Y1_DIR 32 //25 for A-bot

#define Y2_STEP 4 // motor 3
#define Y2_DIR 25 //32 for A-bot

#define motorInterfaceType 1

#define SLEEP1 19
#define SLEEP2 23

#define MICROSTEP 14

#define BUTTON_PIN 13

#define WHEELDIAMETER 7.2
// #define ROBOTDIAMETER 20
#define TILE_LENGTH 50

// #define MAXSPEED 1000
// #define MAXACCELERATION 600

#define MAXSPEED 1000
#define MAXACCELERATION 600

#define MAXTURNSPEED 1500
#define MAXTURNACCELERATION 1250

// #define NOMINAL_SPEED 800
// #define NOMINAL_ACCELERATION 400

#define NOMINAL_SPEED 1500
#define NOMINAL_ACCELERATION 1250

#define MICROSTEP 14

#ifdef USING_TELNET
    #define PRINTER TelnetStream
#else
    #define PRINTER Serial
#endif

#define TURN_KP 20.0
#define TURN_KI 0.0
#define TURN_KD 0.0
#define TURN_DT 0.02

#define ECHO_PIN 18
#define TRIG_PIN 5