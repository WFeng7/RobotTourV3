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

double extra_time = 0;
double predicted_time = 0;
double target_time = 0;

AccelStepper y_stepper1 = AccelStepper(motorInterfaceType, Y1_STEP, 2);
AccelStepper y_stepper2 = AccelStepper(motorInterfaceType, Y2_STEP, 2);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Pathfinding pathfinding = Pathfinding();

std::vector<std::string> path, temp;

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
  PRINTER.println("Entering Normal Mode");
  drivetrain.bno.enterNormalMode();
  delay(10);
  PRINTER.println("Setting External Crystal");
  drivetrain.bno.setExtCrystalUse(true);
  delay(10);
  PRINTER.println("Setting Auto Range");
  drivetrain.bno.enableAutoRange(true);
  delay(10);

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
  pathfinding.setStart({1, 'v'});
  pathfinding.setTarget({0, 0});
  pathfinding.setLastGate({2, 1}); // remove if we don't want last gate.

  std::vector<std::string> vblocks, hblocks;
  pathfinding.addGate("0 2");
  pathfinding.addGate("2 3");
  pathfinding.addGate("3 0");
  pathfinding.addGate("3 3");
  pathfinding.addGate("4 0");

  vblocks.push_back("0110");
  vblocks.push_back("1000");
  vblocks.push_back("0010");
  vblocks.push_back("1011");

  hblocks.push_back("00010");
  hblocks.push_back("00100");
  hblocks.push_back("00000");

  pathfinding.addBlocks(vblocks, hblocks);

  pathfinding.findPath();

  path = pathfinding.getPath();

  pathfinding.gridPath();

  temp = pathfinding.getPath();

  for(std::string &s : temp) {
    path.push_back(s);
  }

  temperature = 27;

  target_time = 79;
  target_time *= 1000;

  extra_time = 0;
  extra_time *= 1000;

  // END CHANGE

  // temperature = bno.getTemp();
  PRINTER.println(temperatureRead());

  usleep(5000*1000);

  PRINTER.println("Setup Complete");

  led.rainbow(false);
  led.setGreen();
}

void run() {
  int tct = 0, cct = 0, rct = 0, lct = 0, ct = (int)path.size();
  for(std::string &s : path) {
    if(s[0] == 't') {
      tct++;
    }
    else if(s[0] == 'c') {
      cct++;
    }
    else if(s[0] == 'r') {
      rct++;
    }
    else if(s[0] == 'l') {
      lct++;
    }
  }
  

  // CHANGE THIS FOR END PATH INSTRUCTIONS


  path.push_back("l");
  path.push_back("p24");


  // predicted_time = 1238 + 1375 * (tct - 1) + 1061 * rct + 1062 * lct + cct * 300;
  predicted_time = 1238 + 1375 * (tct - 1) + 1400 * rct + 1400 * lct + cct * 300; // For Slow Turns
  extra_time = max(target_time - predicted_time, 0.0); // change to 0 if less
  int prev = millis();
  int steps = 0;
  for(std::string &s : path) {
    if(s[0] == 't') {
      drivetrain.driveDistance(std::stod(s.substr(1)), false);
      if(steps == 0) {
        predicted_time -= 1238;
      }
      else {
        predicted_time -= 1375;
      }
    }
    else if(s[0] == 'c') {
      drivetrain.driveDistance(HCSR04.measureDistanceCm()[0] - 17 + 1.75, false); // remember to add offset from half of the wood block; measure it
      predicted_time -= 300;
    }
    else if(s[0] == 'r') {
      drivetrain.turnRight();
      predicted_time -= 1061;
    }
    else if(s[0] == 'l') {
      drivetrain.turnLeft();
      predicted_time -= 1061;
    }
    else if(s[0] == 'a') {
      drivetrain.turnAround();
    }
    else if(s[0] == 'p') {
      drivetrain.driveDistance(HCSR04.measureDistanceCm()[0] - std::stod(s.substr(1)), false);
      predicted_time -= 300;
    }
    double elapsed = millis() - prev;
    extra_time = max((target_time - elapsed) - predicted_time, 0.0);
    delay((int)extra_time/(int)path.size());
    ct--;
  }
  // drivetrain.driveDistance(10, false);
  // drivetrain.turnLeft();
  // drivetrain.driveDistance(HCSR04.measureDistanceCm()[0] - 24, false);
  // drivetrain.turnLeft();
  // drivetrain.driveDistance(HCSR04.measureDistanceCm()[0] - 76, false);
  // drivetrain.driveDistance(7, false);
  // drivetrain.correctWithGyro(drivetrain.getOrientation(), 24.13);
  // drivetrain.turnLeft();
  // drivetrain.driveDistance(15.00, false)
  // drivetrain.driveDistance(HCSR04.measureDistanceCm()[0] - 17 + 1.75, false);
  // drivetrain.turnRight();
  // drivetrain.driveDistance(HCSR04.measureDistanceCm()[0] - 25 + 0.75, false);
}

void loop() {
  digitalWrite(2, HIGH);
  int reading = digitalRead(BUTTON_PIN);
  delay(10);
  PRINTER.println(reading);
  if(reading == HIGH) {
    PRINTER.println("Loop!");
    button_pressed = true;
    led.setBlue();
  }
  else {
    if (reading == LOW && button_pressed) {
      button_pressed = false;

      drivetrain.resetOrientation();
      PRINTER.printf("Pre-zero: %f\n", drivetrain.getYaw());
      double pre = drivetrain.getYaw();
      drivetrain.zeroYaw();
      PRINTER.printf("Post-zero: %f\n", drivetrain.getYaw());
      double post = drivetrain.getYaw();
      PRINTER.print("OK");

      while (!drivetrain.bno.begin(OPERATION_MODE_NDOF)) {
        PRINTER.println("Failed to initialize IMU!");
      }

      // led.rainbow(true);
      drivetrain.bno.enterNormalMode();
      drivetrain.bno.setExtCrystalUse(true);
      drivetrain.bno.enableAutoRange(true);
      // led.rainbow(false);

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