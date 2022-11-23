#include <Arduino.h>

#include "esp32-hal-gpio.h"
#include "distanceSensor.hh"

DistanceSensor::DistanceSensor(int trigPin, int echoPin)
{
    this->trigPin = trigPin;
    this->echoPin = echoPin;

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float DistanceSensor::readDistance() {
    digitalWrite(this->trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trigPin, LOW);

    float duration = pulseIn(this->echoPin, HIGH);
    float distance = duration * 0.017;
    return distance;
}
