
#include "DeadReckoner.h"
#include <Arduino.h>

#define UNSIGNED_LONG_MAX 4294967295

DeadReckoner::DeadReckoner(volatile unsigned long *left, volatile unsigned long *right, double tpr, double r, double l) {
	leftTicks = left;
	rightTicks = right;
	ticksPerRev = tpr;
	radius = r;
	length = l;
}

void DeadReckoner::setX(double x) {
	xc = x;
}

void DeadReckoner::setY(double y) {
	yc = y;
}

void DeadReckoner::setTheta(double t) {
	theta = t;
}

double DeadReckoner::getX() {
	return xc;
}

double DeadReckoner::getY() {
	return yc;
}

double DeadReckoner::getWl() {
	return wl;
}

double DeadReckoner::getWr() {
	return wr;
}

double DeadReckoner::getTheta() {
	return theta;
}

void DeadReckoner::computeAngularVelocities() {
	// Time elapsed after computing the angular velocity previously.
	unsigned long dt_omega = getChange(prevWheelComputeTime, micros()); // in microseconds

	double c = 2 * PI / (ticksPerRev * dt_omega / 1000000.0); // ticks to rad/s conversion factor

	double changeLeftTicks = getChange(*leftTicks, leftTicksPrev);
	double changeRightTicks = getChange(*rightTicks, rightTicksPrev);

	wl = changeLeftTicks * c;
	wr = changeRightTicks * c;

	leftTicksPrev = *leftTicks;
	rightTicksPrev = *rightTicks;

	prevWheelComputeTime = micros();
}

unsigned long DeadReckoner::getChange(unsigned long current, unsigned long previous) {
	// Overflow has occured
	if (current < previous) {
		return UNSIGNED_LONG_MAX - previous + current;
		// Debug info
		// Serial.print("OVERFLOW: "); Serial.print("\tcurrent: "); Serial.print(current);
		// Serial.print("\tprevious: "); Serial.println(previous);
	}

	// No overflow
	return current - previous;
}

void DeadReckoner::computePosition() {
	computeAngularVelocities();
	// Time elapsed after the previous position has been integrated.
	unsigned long dt_integration = getChange(prevIntegrationTime, micros());

	float dt = dt_integration / 1000000.0; // convert to seconds

	// Dead reckoning equations

	float Vl = wl * radius;
	float Vr = wr * radius;
	float v = (Vr + Vl) / 2.0;
	float w = (Vr - Vl) / length;
	// Uses 4th order Runge-Kutta to integrate numerically to find position.
	float xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(theta + dt * w / 2) / 3;
	float yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(theta + dt * w / 2) / 3;
	float thetaNext = theta + dt * w;

	xc = xNext;
	yc = yNext;
	theta = thetaNext;

	prevIntegrationTime = micros();
}
