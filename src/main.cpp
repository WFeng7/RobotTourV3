#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>

#include "Drivetrain.h"
#include <math.h>
#include <Adafruit_BNO055.h>
#include <Pathfinding.h>
#include <HCSR04.h>

#include <Definitions.h>

#include <EEPROM.h>

#include <LED.h>

// Global Variables

int t = 0;

const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

adafruit_bno055_offsets_t bno_offsets;

int lastButtonState = HIGH;
int buttonState;

bool flipped = false;

double temperature = 20;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int dx[4] = {0, 1, 0, -1};
int dy[4] = {1, 0, -1, 0};

int extra_time = 0;

AccelStepper y_stepper1 = AccelStepper(motorInterfaceType, Y1_STEP, 2);
AccelStepper y_stepper2 = AccelStepper(motorInterfaceType, Y2_STEP, 2);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Pathfinding pathfinding = Pathfinding();

std::vector<std::string> path;

Drivetrain drivetrain = Drivetrain(&y_stepper1, &y_stepper2, &bno);

LED led;

bool button_pressed = false;

// Telnet
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

#ifdef USING_TELNET
  const char ssid[] = SECRET_SSID;
  const char pass[] = SECRET_PASS;
#endif

void setup() {
  digitalWrite(2, LOW);
  Serial.begin(115200);

  Wire.begin();
  led.begin(RED_PIN, GREEN_PIN, BLUE_PIN);

  led.rainbow(true);

  usleep(2000*1000);
  Serial.println("Setup");

  #ifdef USING_TELNET
    Serial.printf("Connecting to WiFi: %s\n", ssid);
    WiFi.begin(ssid, pass);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("WiFi Failed!");
      while (1) {
        usleep(10*1000);
      }
    }

    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
    time_t now = time(nullptr);
    while (now < SECS_YR_2000) {
      usleep(100*1000);
      now = time(nullptr);
    }
    setTime(now);

    IPAddress ip = WiFi.localIP();
    Serial.println();
    Serial.println("Connected to WiFi network.");
    Serial.print("Connect with Telnet client to ");
    Serial.println(ip);

    TelnetStream.begin();
  #endif
  HCSR04.begin(TRIG_PIN, ECHO_PIN);

  pinMode(SLEEP1, OUTPUT);
  pinMode(SLEEP2, OUTPUT);
  pinMode(MICROSTEP, OUTPUT);
  digitalWrite(SLEEP1, LOW);
  digitalWrite(SLEEP2, LOW);
  digitalWrite(MICROSTEP, LOW);

  pinMode(Y1_STEP, OUTPUT);
  pinMode(Y1_DIR, OUTPUT);
  pinMode(Y2_STEP, OUTPUT);
  pinMode(Y2_DIR, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  PRINTER.println("IMU Setup");
  if (!drivetrain.bno.begin(OPERATION_MODE_NDOF)) {
    PRINTER.println("Failed to initialize IMU!");
    while (1);
  }

  drivetrain.bno.enterNormalMode();
  drivetrain.bno.setExtCrystalUse(true);
  drivetrain.bno.enableAutoRange(true);

    //calibrate IMU
  PRINTER.println("Calibrating IMU");
  uint8_t system, gyro, accel, mag = 0;

  //blink LED while calibrating

  adafruit_bno055_offsets_t calibrationData;
  #ifdef SAVE_CALIBRATION
    led.rainbow(false);
    while (!(system == 3 && gyro == 3 && mag == 3)) {
      drivetrain.bno.getCalibration(&system, &gyro, &accel, &mag);
      double temp = bno.getTemp();
      PRINTER.printf("Sys: %d, Gyro: %d, Accel: %d, Mag: %d Temp: %d\n", system, gyro, accel, mag);
      usleep(500*1000);
    }
    drivetrain.bno.getSensorOffsets(calibrationData);
    for (unsigned int i = 0; i < sizeof(calibrationData); i++) {
      double percent = ((double)i / (double)sizeof(calibrationData)) * 100.0;
      PRINTER.printf("Writing (%d%)\n", percent);
      EEPROM.write(i, *((uint8_t*)&calibrationData + i));
    }
  #else
    for (unsigned int i = 0; i < sizeof(calibrationData); i++) {
        *((uint8_t*)&calibrationData + i) = EEPROM.read(i);
    }

    bno.setSensorOffsets(calibrationData);
  #endif

  // CHANGE THIS:
  pathfinding.setStart(3);
  pathfinding.setTarget({1, 3});

  std::vector<std::string> vblocks, hblocks;
  pathfinding.addGate("1 0");
  pathfinding.addGate("0 2");
  pathfinding.addGate("2 3");
  // pathfinding.addGate("3 3");
  vblocks.push_back("010");
  vblocks.push_back("100");
  vblocks.push_back("001");
  vblocks.push_back("010");

  hblocks.push_back("0000");
  hblocks.push_back("1110");
  hblocks.push_back("0100");

  temperature = 23;

  extra_time = 8000;

  pathfinding.addBlocks(vblocks, hblocks);

  pathfinding.findPath();

  path = pathfinding.getPath();

  // temperature = bno.getTemp();
  PRINTER.println(temperatureRead());

  usleep(5000*1000);

  PRINTER.println("Setup Complete");

  led.rainbow(false);
  led.setGreen();
}

void run() {
  for(std::string &s : path) {
    if(s[0] == 't') {
      drivetrain.driveDistance(std::stoi(s.substr(1)), false);
    }
    else if(s[0] == 'r') {
      drivetrain.turnRight();
    }
    else if(s[0] == 'l') {
      drivetrain.turnLeft();
    }
    else if(s[0] == 'a') {
      drivetrain.turnAround();
    }
    delay(extra_time/((int)path.size()));
  }
  drivetrain.correctWithGyro(drivetrain.getOrientation(), 24.13);
  drivetrain.driveDistance(HCSR04.measureDistanceCm()[0] - 17 + 3, false);
  drivetrain.turnRight();
  drivetrain.driveDistance(HCSR04.measureDistanceCm()[0] - 75 + 4, false);
  // drivetrain.driveDistance(200, false);
  // drivetrain.correctWithGyro(0, 24.13);
}

void loop() {
  digitalWrite(2, HIGH);
  int reading = digitalRead(BUTTON_PIN);
  delay(10);

  if(reading == LOW) {
    button_pressed = true;
    led.setBlue();
  }
  else {
    if (reading == HIGH && button_pressed) {
      button_pressed = false;

      drivetrain.resetOrientation();
      PRINTER.printf("Pre-zero: %f\n", drivetrain.getYaw());
      double pre = drivetrain.getYaw();
      drivetrain.zeroYaw();
      PRINTER.printf("Post-zero: %f\n", drivetrain.getYaw());
      double post = drivetrain.getYaw();
      PRINTER.print("OK");

      led.setYellow();

      delay(2000);

      led.setRed();
      run();

      drivetrain.stepperSleep();
      digitalWrite(2, LOW);
      led.setGreen();
    }
  }
}