
#ifndef _DeadReckoner_h
#define _DeadReckoner_h

#include <Arduino.h>

#define UNSIGNED_LONG_MAX 4294967295
#define WHEEL_FORWARD 1
#define WHEEL_REVERSE -1

class DeadReckoner {

public:
	DeadReckoner(volatile unsigned long *, volatile unsigned long *, double, double, double);
	void computePosition();
	void setX(double);
	void setY(double);
	void setTheta(double);
	void setLeftOmegaDirection(int);
	void setRightOmegaDirection(int);
	int getLeftOmegaDirection();
	int getRightOmegaDirection();
	double getX();
	double getY();
	double getW();
	double getWl();
	double getWr();
	double getTheta();

private:
	void computeAngularVelocities();
	unsigned long getChange(unsigned long, unsigned long);
	volatile unsigned long *leftTicks, *rightTicks; // Number of total wheel encoder tick counts for left and right wheels.
	unsigned long leftTicksPrev, rightTicksPrev; // Number of total wheel encoder tick counts at time computeAngularVelocities() is called.
	double xc, yc; // Latest position coordinates in ticks.
	double wl, wr; // Latest left and right angular velocity of the wheels in radians per second.
	double ticksPerRev; // Number of tick registers per second of the encoder.
	float w; // Angular velocity of the robot.
	double length; // Length from left wheel to right wheel.
	double radius; // Radius of the wheel.
	double theta;
	double toRadPerSec; // ticks/microsecond to rad/s conversion factor
	unsigned long prevIntegrationTime;
	unsigned long prevWheelComputeTime;
	int leftOmegaDirection = 1;
	int rightOmegaDirection = 1;
	unsigned long dt_omega;
	unsigned long dt_integration;
};

#endif
