#ifndef _LIGHT
#define _LIGHT

class Light {

    public:
        Light(int pin);
        ~Light();
        void turnOn();
        void turnOff();
        bool isOn();
        void configurePins();

    private:
        bool state;
        int led_pin;
        bool pinsConfigured;
};

#endif // _LIGHT