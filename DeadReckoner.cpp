/*
 Name:		DeadReckoner.cpp
 Created:	5/18/2018 4:58:56 PM
 Author:	while
 Editor:	http://www.visualmicro.com
*/

#include "DeadReckoner.h"
#include <Arduino.h>

#define UNSIGNED_LONG_MAX 4294967295

DeadReckoner::DeadReckoner(int& left, int& right, double tpr, double r, double l) {
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

void DeadReckoner::computeAngularVelocities() {
	unsigned long dt_omega = micros() - prevWheelComputeTime; // in microseconds
	if (dt_omega < 0) {
		// micros() has overflowed and reset to 0
		dt_omega = UNSIGNED_LONG_MAX - prevWheelComputeTime + micros();
	}
	float c = 2 * PI / (ticksPerRev * dt_omega / 1000000.0); // ticks to rad/s conversion factor
	wl = (leftTicks - leftTicksPrev) * c;
	wr = (rightTicks - rightTicksPrev) * c;
	
	leftTicksPrev = leftTicks;
	rightTicksPrev = rightTicks;

	prevWheelComputeTime = micros();
}

void DeadReckoner::integratePosition() {
	unsigned long dt_integration = micros() - prevIntegrationTime;
	if (dt_integration < 0) {
		// micros() has overflowed and has reset to 0
		dt_integration = UNSIGNED_LONG_MAX - prevIntegrationTime + micros();
	}

	float dt = dt_integration / 1000000.0; // convert to seconds

	float Vl = wl * radius;
	float Vr = wr * radius;
	float v = (Vr + Vl) / 2.0;
	float w = (Vr - Vl) / length;

	float xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(theta + dt * w / 2) / 3;
	float yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(theta + dt * w / 2) / 3;
	float thetaNext = theta + dt * w;

	xc = xNext;
	yc = yNext;
	theta = thetaNext;

	prevIntegrationTime = micros();
}
