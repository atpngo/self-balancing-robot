#include "light.h"
#include "Arduino.h"

Light::Light(int pin) {
    pinsConfigured = false;
    led_pin = pin;
    configurePins();
}

Light::~Light() {

}

void Light::configurePins() {
    pinMode(led_pin, OUTPUT);
    pinsConfigured = true;
}

void Light::turnOn() {
    state = true;
    digitalWrite(led_pin, HIGH);
}

void Light::turnOff() {
    state = false;
    digitalWrite(led_pin, LOW);
}

bool Light::isOn() {
    return state;
}
