
#include "DeadReckoner.h"

DeadReckoner::DeadReckoner(volatile unsigned long *left, volatile unsigned long *right, int tpr, double r, double l) {
	leftTicks = left;
	rightTicks = right;
	ticksPerRev = tpr;
	radius = r;
	length = l;
	toRadPerSec = 1000000.0 * TWO_PI / ticksPerRev;
}


void DeadReckoner::setParams(volatile unsigned long *left, volatile unsigned long *right, double tpr, double r, double l) {
	leftTicks = left;
	rightTicks = right;
	ticksPerRev = tpr;
	radius = r;
	length = l;
	toRadPerSec = 1000000 * TWO_PI / ticksPerRev;
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
	// TODO: Check this
	return wl;
}

double DeadReckoner::getWr() {
	// TODO: Check this
	return wr;
}

double DeadReckoner::getW() {
	return w;
}

double DeadReckoner::getTheta() {
	return theta;
}

double DeadReckoner::get_v() {
	return v;
}

double DeadReckoner::get_w() {
	return w;
}

void DeadReckoner::setLeftOmegaDirection(int8_t direction) {
	leftOmegaDirection = direction;
}

void DeadReckoner::setRightOmegaDirection(int8_t direction) {
	rightOmegaDirection = direction;
}

int8_t DeadReckoner::getLeftOmegaDirection() {
	return leftOmegaDirection;
}

int8_t DeadReckoner::getRightOmegaDirection() {
	return rightOmegaDirection;
}



void DeadReckoner::computeAngularVelocities() {
	// Time elapsed after computing the angular velocity previously.
	// change in time is defined as previous - current to prevent round off error.
	dt_omega = getChange(micros(), prevWheelComputeTime); // in milliseconds

	double changeLeftTicks = getChange(*leftTicks, leftTicksPrev);
	double changeRightTicks = getChange(*rightTicks, rightTicksPrev);

	wl = (leftOmegaDirection * changeLeftTicks * toRadPerSec) / dt_omega ;
	wr = (rightOmegaDirection * changeRightTicks * toRadPerSec) / dt_omega ;

	
	// Serial.print("\twl: "); Serial.print(wl*toRPM, 5);
	// Serial.print("\twr: "); Serial.print(wr*toRPM, 5);
	// Serial.print("\tdt: "); Serial.print(dt_omega);
	// Serial.print("\tlt: "); Serial.print(changeLeftTicks);
	// Serial.print("\trt: "); Serial.println(changeRightTicks);

	leftTicksPrev = *leftTicks;
	rightTicksPrev = *rightTicks;

	prevWheelComputeTime = micros();
}

void DeadReckoner::computePosition() {
	if (micros() - prevIntegrationTime > positionComputeInterval) {
		computeAngularVelocities();
		// Time elapsed after the previous position has been integrated.
		// change in time is defined as previous - current to prevent round off error.
		dt_integration = getChange(micros(), prevIntegrationTime);

		float dt = dt_integration / 1000000.0; // convert to seconds

		// Dead reckoning equations

		float Vl = wl * radius;
		float Vr = wr * radius;
		v = (Vr + Vl) / 2.0;
		w = (Vr - Vl) / length;
		// Uses 4th order Runge-Kutta to integrate numerically to find position.
		float xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(theta + dt * w / 2) / 3;
		float yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(theta + dt * w / 2) / 3;
		float thetaNext = theta + dt * w;

		xc = xNext;
		yc = yNext;
		theta = thetaNext;

		// float toRPM = 30 / PI;
		// float dist = sqrt(xc*xc + yc * yc);
		// Serial.print("\tdist: "); Serial.print(dist);
		// Serial.print("\twl: "); Serial.print(wl*toRPM, 5);
		// Serial.print("\twr: "); Serial.print(wr*toRPM, 5);
		// Serial.print("\tVl: "); Serial.print(Vl);
		// Serial.print("\tVr: "); Serial.print(Vr);
		// Serial.print("\tw: "); Serial.print(w, 5);
		// Serial.print("\tx: "); Serial.print(xc);
		// Serial.print("\ty: "); Serial.print(yc);
		// Serial.print("\ttheta: "); Serial.println(theta*RAD_TO_DEG);

		prevIntegrationTime = micros();
	}
}
