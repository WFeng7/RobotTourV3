#ifndef LED_h
#define LED_h

#include <thread>
#include <atomic>

 

class LED {
    private:
       int redPin, greenPin, bluePin;

       std::atomic<bool> running;
       std::thread t;

    public:
        LED();
        void begin(int redPin, int greenPin, int bluePin);
        void setColor(int red, int green, int blue);
        void setRed();
        void setGreen();
        void setBlue();
        void setYellow();
        void rainbow(bool state);
        void rainbowThread();
        void hsvToRgb(float h, float s, float v, int *r, int *g, int *b);
        void off();
};

#endif