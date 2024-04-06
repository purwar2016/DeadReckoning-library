
#ifndef _DeadReckoner_h
#define _DeadReckoner_h

#include <Arduino.h>

#define UNSIGNED_LONG_MAX 4294967295

class DeadReckoner {

public:
	DeadReckoner(volatile unsigned long *, volatile unsigned long *, int, double, double);
	void setParams(volatile unsigned long *, volatile unsigned long *, double, double, double);
	void computePosition();
	void setX(double);
	void setY(double);
	void setTheta(double);
	void setLeftOmegaDirection(int8_t);
	void setRightOmegaDirection(int8_t);
	int8_t getLeftOmegaDirection();
	int8_t getRightOmegaDirection();
	double getX();
	double getY();
	double getW();
	double getWl();
	double getWr();
	double getTheta();
	double get_v();
	double get_w();

private:
	void computeAngularVelocities();
	volatile unsigned long *leftTicks, *rightTicks; // Number of total wheel encoder tick counts for left and right wheels.
	unsigned long leftTicksPrev, rightTicksPrev; // Number of total wheel encoder tick counts at time computeAngularVelocities() is called.
	double xc, yc; // Latest position coordinates in ticks.
	double wl, wr; // Latest left and right angular velocity of the wheels in radians per second.
	int ticksPerRev; // Number of tick registers per second of the encoder.
	float w; // Angular velocity of the robot in rad/s
	float v; //for- backwards velocity of robot mm/s
	double length; // Length from left wheel to right wheel.
	double radius; // Radius of the wheel in mm
	double theta; // in rad
	double toRadPerSec; // ticks/microsecond to rad/s conversion factor
	unsigned long prevIntegrationTime;
	unsigned long prevWheelComputeTime;
	unsigned long positionComputeInterval;
	int8_t leftOmegaDirection = 1;
	int8_t rightOmegaDirection = 1;
	unsigned long dt_omega;
	unsigned long dt_integration;
	const double wrap_angle(double angle);

	unsigned long static getChange(unsigned long current, unsigned long previous) {
		// Overflow has occured
		if (current < previous) {
			return UNSIGNED_LONG_MAX - previous + current;
		}
		// No overflow
		return current - previous;
	}
};

#endif
