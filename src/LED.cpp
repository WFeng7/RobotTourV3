#include <LED.h>
#include <Definitions.h>
#include <Arduino.h>
#include <thread>

LED::LED() {}

void LED::begin(int redPin, int greenPin, int bluePin) {
    this->redPin = redPin;
    this->greenPin = greenPin;
    this->bluePin = bluePin;

    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    setColor(0, 0, 0);
}

void LED::setColor(int red, int green, int blue) {
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
}

void LED::off() {
    setColor(0, 0, 0);
}

void LED::setRed() {
    setColor(255, 0, 0);
}

void LED::setGreen() {
    setColor(0, 255, 0);
}

void LED::setBlue() {
    setColor(0, 0, 255);
}

void LED::setYellow() {
    setColor(255, 255, 0);
}

void LED::rainbow(bool state) {
    if (state) {
        running = true;
        t = std::thread(&LED::rainbowThread, this);
    }
    else {
        running = false;
        // if (t.joinable()) {
        //     t.join();
        // }
    }
}

void LED::rainbowThread() {
    int red = 254, green = 1, blue = 127;
    int red_dir = -1;
    int green_dir = 1;
    int blue_dir = -1;
    while (running) {
        red = red + red_dir;
        green = green + green_dir;
        blue = blue + blue_dir;

        if (red >= 255 || red <= 0) {
            red_dir = -red_dir;
        }
        if (green >= 255 || green <= 0) {
            green_dir = -green_dir;
        }
        if (blue >= 255 || blue <= 0) {
            blue_dir = -blue_dir;
        }
        setColor(red, green, blue);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void LED::hsvToRgb(float h, float s, float v, int* r, int* g, int* b) {
    float f = h / 60.0f;
    int i = floor(f);
    float pD = v * (1 - s);
    float qD = v * (1 - s * (f - i));
    float tD = v * (1 - s * (1 - (f - i)));

    int p = (int) pD;
    int q = (int) qD;
    int t = (int) tD;

    switch(i) {
        case 0:
            *r = v * 255; *g = t * 255; *b = p * 255;
            break;
        case 1:
            *r = q * 255; *g = v * 255; *b = p * 255;
            break;
        case 2:
            *r = p * 255; *g = v * 255; *b = t * 255;
            break;
        case 3:
            *r = p * 255; *g = q * 255; *b = v * 255;
            break;
        case 4:
            *r = t * 255; *g = p * 255; *b = v * 255;
            break;
        default:
            *r = v * 255; *g = p * 255; *b = q * 255;
            break;
    }
}


