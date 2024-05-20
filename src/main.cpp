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

// Global Variables

int t = 0;

const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

adafruit_bno055_offsets_t bno_offsets;

int lastButtonState = HIGH;
int buttonState;

bool flipped = false;

float temperature = 20;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int dx[4] = {0, 1, 0, -1};
int dy[4] = {1, 0, -1, 0};

AccelStepper y_stepper1 = AccelStepper(motorInterfaceType, Y1_STEP, 2);
AccelStepper y_stepper2 = AccelStepper(motorInterfaceType, Y2_STEP, 2);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Pathfinding pathfinding = Pathfinding();

std::vector<std::string> path;

Drivetrain drivetrain = Drivetrain(&y_stepper1, &y_stepper2, &bno);

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
  delay(2000);
  Serial.println("Setup");

  #ifdef USING_TELNET
    Serial.printf("Connecting to WiFi: %s\n", ssid);
    WiFi.begin(ssid, pass);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("WiFi Failed!");
      while (1) {
        delay(10);
      }
    }

    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
    time_t now = time(nullptr);
    while (now < SECS_YR_2000) {
      delay(100);
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
    while (!(system == 3 && accel == 3 && gyro == 3 && mag == 3)) {
      digitalWrite(2, HIGH);
      drivetrain.bno.getCalibration(&system, &gyro, &accel, &mag);
      PRINTER.printf("Sys: %d, Gyro: %d, Accel: %d, Mag: %d\n", system, gyro, accel, mag);
      delay(2000);
      digitalWrite(2, LOW);
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
  pathfinding.setStart(0);
  pathfinding.setTarget({1, 3});

  std::vector<std::string> vblocks, hblocks;
  pathfinding.addGate("0 1");
  pathfinding.addGate("2 0");
  pathfinding.addGate("3 0");
  pathfinding.addGate("3 3");
  // pathfinding.addWood(0, 0, 0, 1);
  // pathfinding.addWood(0, 1, 1, 1);
  // pathfinding.addWood(1, 3, 1, 2);
  // pathfinding.addWood(1, 3, 2, 3);
  // pathfinding.addWood(1, 0, 2, 0);
  // pathfinding.addWood(2, 0, 3, 0);
  // pathfinding.addWood(2, 1, 3, 1);
  // pathfinding.addWood(3, 2, 3, 3);
  vblocks.push_back("010");
  vblocks.push_back("000");
  vblocks.push_back("101");
  vblocks.push_back("011");

  hblocks.push_back("0101");
  hblocks.push_back("0000");
  hblocks.push_back("1000");

  pathfinding.addBlocks(vblocks, hblocks);

  pathfinding.findPath();

  path = pathfinding.getPath();

  temperature = 20;
  // temperature = bno.getTemp();

  delay(5000);

  PRINTER.println("Setup Complete");
}

std::vector<std::string> split(const std::string& s, char c) {
  std::vector<std::string> result;
  size_t begin = 0;
  while (true) {
    size_t end = s.find_first_of(c, begin);
    result.push_back(s.substr(begin, end - begin));

    if (end == std::string::npos) {
      break;
    }

    begin = end + 1;
  }
  return result;
}

void run() {
  for(std::string &s : path) {
    if(s[0] == 't') {
      // drivetrain.driveDistance((flipped ? -1 : 1) * std::stoi(s.substr(1)), false);
      drivetrain.driveDistance(std::stoi(s.substr(1)), false);
      // Serial.println("Drive distance");
    }
    else if(s[0] == 'r') {
      drivetrain.turnRight();
    }
    else if(s[0] == 'l') {
      drivetrain.turnLeft();
    }
    else if(s[0] == 'a') {
      drivetrain.turnAround();
      // flipped = !flipped;
    }
  }
  // double* distances = HCSR04.measureDistanceCm(temperature); 
  // drivetrain.driveDistance(distances[0] - #, false);
  // drivetrain.turnRight();
  // drivetrain.driveDistance(distances[0] - #, false);
}

void loop() {
  digitalWrite(2, HIGH);
  int reading = digitalRead(BUTTON_PIN);

  if(reading == LOW) {
    drivetrain.resetOrientation();
    PRINTER.printf("Pre-zero: %f\n", drivetrain.getYaw());
    drivetrain.zeroYaw();
    PRINTER.printf("Post-zero: %f\n", drivetrain.getYaw());
    PRINTER.print("OK");
    delay(2000);
    run();
    drivetrain.stepperSleep();
    digitalWrite(2, LOW);
    delay(1000);
  }
}