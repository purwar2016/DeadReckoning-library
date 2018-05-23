
#include "DeadReckoner.h"
#include <Arduino.h>

#define UNSIGNED_LONG_MAX 4294967295

DeadReckoner::DeadReckoner(volatile int *left, volatile int *right, double tpr, double r, double l) {
	leftTicks = left;
	rightTicks = right;
	ticksPerRev = tpr;
	radius = r;
	length = l;
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
	unsigned long dt_omega = micros() - prevWheelComputeTime; // in microseconds
	if (dt_omega < 0) {
		// micros() has overflowed and reset to 0
		dt_omega = UNSIGNED_LONG_MAX - prevWheelComputeTime + micros();
	}

	float c = 2 * PI / (ticksPerRev * dt_omega / 1000000.0); // ticks to rad/s conversion factor
	wl = (*leftTicks - leftTicksPrev) * c;
	wr = (*rightTicks - rightTicksPrev) * c;
	
	leftTicksPrev = *leftTicks;
	rightTicksPrev = *rightTicks;

	prevWheelComputeTime = micros();
}

void DeadReckoner::computePosition() {
	computeAngularVelocities();
	// Time elapsed after the previous position has been integrated.
	unsigned long dt_integration = micros() - prevIntegrationTime;
	if (dt_integration < 0) {
		// micros() has overflowed and has reset to 0
		dt_integration = UNSIGNED_LONG_MAX - prevIntegrationTime + micros();
	}

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
