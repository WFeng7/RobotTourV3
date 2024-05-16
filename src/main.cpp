#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ContinuousStepper.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
// #include <MPU9250.h>

#include "Drivetrain.h"
#include <math.h>
#include <Adafruit_BNO055.h>
// #include "Constants.h"

// Pins

#define X1_STEP 27 // motor 4
#define X1_DIR 18

#define X2_STEP 27 // motor 2
#define X2_DIR 15

#define Y1_STEP 4 // motor 1
#define Y1_DIR 32

#define Y2_STEP 4 // motor 3
#define Y2_DIR 25

#define motorInterfaceType 1

#define SLEEP1 19
#define SLEEP2 23

#define MICROSTEP 14

#define BUTTON_PIN 13

// Global Variables

int t = 0;

const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

adafruit_bno055_offsets_t bno_offsets;

int lastButtonState = HIGH;
int buttonState;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

AccelStepper x_stepper1 = AccelStepper(motorInterfaceType, X1_STEP, 2);
AccelStepper x_stepper2 = AccelStepper(motorInterfaceType, X2_STEP, 2);
AccelStepper y_stepper1 = AccelStepper(motorInterfaceType, Y1_STEP, 2);
AccelStepper y_stepper2 = AccelStepper(motorInterfaceType, Y2_STEP, 2);

ContinuousStepper<StepperDriver> continous;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Drivetrain drivetrain = Drivetrain(&x_stepper1, &x_stepper2, &y_stepper1, &y_stepper2, &bno, &continous);

const short N = 4;
const short dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
short n_bonus;
short bonus[N * N] = { 0 };
short start;
std::pair<short, short> target;
bool wood[N*N][N*N] = { false };

std::vector<std::string> path;

struct T {
    short x, y, z, p, pz;
};

void addWood(short a, short b, short c, short d) {
  wood[a * N + b][c * N + d] = true;
  wood[c * N + d][a * N + b] = true;
}

void addGate(short a, short b) {
  ++n_bonus;
  bonus[a * N + b] = n_bonus;
}

void findPath() {
  std::pair<short, short> vis[N * N][1 << n_bonus];
  bool flag = false;
  for(short i = 0; i < N*N; i++) {
    for(short j = 0; j < (1 << n_bonus); j++) {
      vis[i][j] = {-1, -1};
    }
  }
  std::queue<T> q;
  T t; t.x = start; t.y = 0; t.z = 0; t.p = -1, t.pz = 0;
  T nt;
  vis[t.x * N][0] = {-1, 0};
  q.push(t);
  while(q.size()) {
      t = q.front();
      q.pop();
      nt.p = t.x * N + t.y;
      nt.pz = t.z;
      for(short i = 0; i < 4; i++) {
        nt.x = dx[i] + t.x;
        nt.y = dy[i] + t.y;
        nt.z = t.z;
        if(bonus[nt.x * N + nt.y]) {
          nt.z |= (1 << (bonus[nt.x * N + nt.y] - 1));
        }
        if(nt.x >= 0 && nt.x < N && nt.y >= 0 && nt.y < N && !wood[nt.p][nt.x * N + nt.y] && vis[nt.x * N + nt.y][nt.z].second == -1) {
          vis[nt.x * N + nt.y][nt.z] = {nt.p, nt.pz};
          if(nt.x == target.first && nt.y == target.second && nt.z == (1 << n_bonus) - 1) {
            flag = true;
            break;
          }
          q.push(nt);
        }
    }
    if(flag) {
        break;
    }
  }
  // final is nt
  std::vector<short> coords;
  std::pair<short, short> tt = {nt.x * N + nt.y, nt.z};
  while(tt.first != -1) {
    coords.push_back(tt.first);
    tt = vis[tt.first][tt.second];
  }
  short a, b, c, d;
  short dist = 36;
  short dir = 0, ndir = 0; // 0 up 1 right 2 down 3 left
  for(short i = coords.size() - 2; i >= 0; i--) {
    a = coords[i + 1] / N;
    b = coords[i + 1] % N;
    c = coords[i] / N;
    d = coords[i] % N;
    if(d - b > 0) {
      ndir = 0;
    }
    else if(c - a > 0) {
      ndir = 1;
    }
    else if(d - b < 0) {
      ndir = 2;
    }
    else {
      ndir = 3;
    } 
    if(dir != ndir) {
      path.push_back("t" + std::to_string(dist));
      dist = 50;
      if(ndir - dir == 1 || ndir - dir == -3) {
        path.push_back("r");
      }
      else if(ndir - dir == -1 || ndir - dir == 3) {
        path.push_back("l");
      }
      else {
        // path.push_back("l");
        // path.push_back("l");
        path.push_back("a");
      }
      dir = ndir;
    }
    else {
      dist += 50;
    }
  }
  if(dist != 0) {
    path.push_back("t" + std::to_string(dist));
  }
}

void setup() {
  digitalWrite(2, LOW);
  Serial.begin(115200);

  Wire.begin();
  delay(2000);
  Serial.println("Setup");

  pinMode(SLEEP1, OUTPUT);
  pinMode(SLEEP2, OUTPUT);
  pinMode(MICROSTEP, OUTPUT);
  digitalWrite(SLEEP1, LOW);
  digitalWrite(SLEEP2, LOW);
  digitalWrite(MICROSTEP, LOW);

  pinMode(X1_STEP, OUTPUT);
  pinMode(X1_DIR, OUTPUT);
  pinMode(X2_STEP, OUTPUT);
  pinMode(X2_DIR, OUTPUT);
  pinMode(Y1_STEP, OUTPUT);
  pinMode(Y1_DIR, OUTPUT);
  pinMode(Y2_STEP, OUTPUT);
  pinMode(Y2_DIR, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("IMU Setup");
  if (!drivetrain.bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  drivetrain.bno.enterNormalMode();
  drivetrain.bno.setExtCrystalUse(true);
  drivetrain.bno.enableAutoRange(true);

  delay(5000);

  // bno_offsets.accel_offset_x = 0;
  // bno_offsets.accel_offset_y = 0;
  // bno_offsets.accel_offset_z = 0;
  // bno_offsets.gyro_offset_x = 0;
  // bno_offsets.gyro_offset_y = 0;
  // bno_offsets.mag_offset_x = 0;
  // bno_offsets.mag_offset_y = 0;
  // bno_offsets.mag_offset_z = 0;
  // drivetrain.imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // drivetrain.imu.setGyroFSR(2000);
  // drivetrain.imu.setAccelFSR(2);
  // drivetrain.imu.setLPF(5);
  // drivetrain.imu.setSampleRate(10);
  // drivetrain.imu.setCompassSampleRate(10);

  // drivetrain.createMutex();
  // xTaskCreatePinnedToCore(sensorTask, "SensorTask", 10000, NULL, 1, &drivetrain.sensorTaskHandle, 1);
  
  // CHANGE THIS:
  start = 0;
  target = {1, 3};
  addGate(0, 1);
  addGate(2, 0);
  addGate(3, 0);
  addGate(3, 3);
  addWood(0, 0, 0, 1);
  addWood(0, 1, 1, 1);
  addWood(1, 3, 1, 2);
  addWood(1, 3, 2, 3);
  addWood(1, 0, 2, 0);
  addWood(2, 0, 3, 0);
  addWood(2, 1, 3, 1);
  addWood(3, 2, 3, 3);
  
  findPath();
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

void test() {
  Serial.println("Enter command: ");
  String command = Serial.readStringUntil('\n');

  while (command == "") {
    command = Serial.readStringUntil('\n');
  }

  std::vector<std::string> commands = split(command.c_str(), ' ');

  Serial.println("Executing: " + command);

  int x = std::stoi(commands[0]);
  double y = std::stod(commands[1]);

  drivetrain.turn(x, y);
}

void run() {
  for(std::string &s : path) {
    if(s[0] == 't') {
      // drivetrain.driveDistance(std::stoi(s.substr(1)), false);
      Serial.println("Drive distance");
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
  }
  // drivetrain.turnRight();
  // drivetrain.turnLeft();
  // drivetrain.turnAround();
  // drivetrain.driveTiles(1, false);

}

void loop() {
  digitalWrite(2, HIGH);
  int reading = digitalRead(BUTTON_PIN);

  if(reading == LOW) {
    drivetrain.resetOrientation();
    Serial.println(drivetrain.getYaw());
    while (fabs(drivetrain.getYaw()) > 0.5) {
      double yaw = drivetrain.getYaw();
      drivetrain.setYawOffset(yaw);
      Serial.println(drivetrain.getYaw());
    }
    Serial.print("OK");
    delay(2000);
    run();
  }
}