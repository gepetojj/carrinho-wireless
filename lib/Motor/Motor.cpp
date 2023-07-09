#include <Arduino.h>

#include "esp32-hal-gpio.h"
#include "Motor.hh"

Motor::Motor(int npPin, int nnPin, int velPin, bool invert)
{
	this->npPin = npPin;
	this->nnPin = nnPin;
	this->velPin = velPin;
	this->invert = invert;

	pinMode(npPin, OUTPUT);
	pinMode(nnPin, OUTPUT);
	pinMode(velPin, OUTPUT);

	// Ativa o motor no driver
	digitalWrite(velPin, HIGH);
}

void Motor::moveForward()
{
	digitalWrite(this->npPin, this->invert ? LOW : HIGH);
	digitalWrite(this->nnPin, this->invert ? HIGH : LOW);
}

void Motor::moveForward(unsigned int speed)
{
	analogWrite(this->velPin, speed);
	this->moveForward();
}

void Motor::moveBackward()
{
	digitalWrite(this->npPin, this->invert ? HIGH : LOW);
	digitalWrite(this->nnPin, this->invert ? LOW : HIGH);
}

void Motor::moveBackward(unsigned int speed)
{
	analogWrite(this->velPin, speed);
	this->moveBackward();
}

void Motor::stopMovement()
{
	digitalWrite(this->npPin, HIGH);
	digitalWrite(this->nnPin, HIGH);
}