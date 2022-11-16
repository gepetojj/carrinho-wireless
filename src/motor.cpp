#include <Arduino.h>

#include "esp32-hal-gpio.h"
#include "motor.h"

Motor::Motor(int npPin, int nnPin, int velPin)
{
	this->npPin = npPin;
	this->nnPin = nnPin;
	this->velPin = velPin;

	pinMode(npPin, OUTPUT);
	pinMode(nnPin, OUTPUT);
	pinMode(velPin, OUTPUT);
}

void Motor::moveForward()
{
	digitalWrite(this->npPin, HIGH);
	digitalWrite(this->nnPin, LOW);

	delay(300);
	this->stopMovement();
}

void Motor::moveForward(unsigned int speed)
{
	analogWrite(this->velPin, speed);
	this->moveForward();
}

void Motor::moveBackward()
{
	digitalWrite(this->npPin, LOW);
	digitalWrite(this->nnPin, HIGH);

	delay(300);
	this->stopMovement();
}

void Motor::moveBackward(unsigned int speed)
{
	analogWrite(this->velPin, speed);
	this->moveBackward();
}

void Motor::stopMovement()
{
	digitalWrite(this->npPin, LOW);
	digitalWrite(this->nnPin, LOW);
}