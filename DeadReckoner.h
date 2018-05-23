/*
Name:		DeadReckoner.h
Created:	5/18/2018 4:58:56 PM
Author:	while
Editor:	http://www.visualmicro.com
*/

#ifndef _DeadReckoner_h
#define _DeadReckoner_h
#include <Arduino.h>

class DeadReckoner {

public:
	DeadReckoner(volatile int *, volatile int *, double, double, double);
	void integratePosition();
	void computeAngularVelocities();
	double getX();
	double getY();
	double getWl();
	double getWr();

private:
	volatile int *leftTicks, *rightTicks; // Number of total wheel encoder tick counts for left and right wheels.
	int leftTicksPrev, rightTicksPrev; // Number of total wheel encoder tick counts at time computeAngularVelocities() is called.
	double xc, yc; // Latest position coordinates in ticks.
	double wl, wr; // Latest left and right angular velocity of the wheels in radians per second.
	double ticksPerRev; // Number of tick registers per second of the encoder.
	double length; // Length from left wheel to right wheel.
	double radius; // Radius of the wheel.
	double theta;
	unsigned long prevIntegrationTime;
	unsigned long prevWheelComputeTime;

};

#endif
