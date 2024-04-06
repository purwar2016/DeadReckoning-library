#include <DeadReckoner.h>

// ENCODER PINS
#define ENCODER_LEFT_PIN 15
#define ENCODER_RIGHT_PIN 4


// MEASUREMENTS
// The units for all measurements must be consistent.
// You can use any length unit as desired.
#define RADIUS 32.5 // wheel radius in mm
#define LENGTH 110. // wheel separation in mm
#define TICKS_PER_REV 40

// TIME INTERVALS
#define POSITION_COMPUTE_INTERVAL 200 // milliseconds
#define SEND_INTERVAL 500			  // milliseconds

// Number of left and right tick counts on the encoder.
volatile unsigned long leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;


DeadReckoner deadReckoner(&leftTicks, &rightTicks, TICKS_PER_REV, RADIUS, LENGTH);


void pulseLeft() { leftTicks++; }
void pulseRight() { rightTicks++; }

/**
Attaches interrupt and disables all serial communications.
This is necessary because when interrupts are active, incoming serial communication can be lost.
*/
void attachInterrupts()
{
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, CHANGE);
}

void setup() {
	attachInterrupts();
	Serial.begin(115200);
}

void loop() {
	if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
		// Computes the new angular velocities and uses that to compute the new position.
		// The accuracy of the position estimate will increase with smaller time interval until a certain point.
		deadReckoner.computePosition();
		prevPositionComputeTime = millis();
	}

	if (millis() - prevSendTime > SEND_INTERVAL) {
		// Cartesian coordinate of latest location estimate.
		// Length unit correspond to the one specified under MEASUREMENTS.
		double x = deadReckoner.getX();
		double y = deadReckoner.getY();

		// forward velocity and angular velocity of the robot.
		double v = deadReckoner.get_v();
		double w = deadReckoner.get_w();
		// Left and right angular velocities.
		double wl = deadReckoner.getWl();
		double wr = deadReckoner.getWr();

		// getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
		// This angle is set initially at zero before the robot starts moving.
		double theta = deadReckoner.getTheta();

		// Total distance robot has traveld.
		double distance = sqrt(x * x + y * y);

		Serial.print("x: "); Serial.print(x);
		Serial.print("\ty: "); Serial.print(y);
		Serial.print("\twl: "); Serial.print(wl);
		Serial.print("\twr: "); Serial.print(wr);
		Serial.print("\ttheta: "); Serial.print(theta*RAD_TO_DEG); // theta converted to degrees.
		Serial.print("\tdist: "); Serial.print(distance);
		Serial.print("\tv: "); Serial.print(v);
		Serial.print("\tw: "); Serial.println(w);

		prevSendTime = millis();
	}
}