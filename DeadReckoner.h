
#ifndef _DeadReckoner_h
#define _DeadReckoner_h

#include <Arduino.h>

class DeadReckoner {

public:
	DeadReckoner(volatile unsigned int *, volatile unsigned int *, double, double, double);
	void computePosition();
	void setX(double);
	void setY(double);
	void setTheta(double);
	double getX();
	double getY();
	double getWl();
	double getWr();
	double getTheta();

private:
	void computeAngularVelocities();
	volatile unsigned int *leftTicks, *rightTicks; // Number of total wheel encoder tick counts for left and right wheels.
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
