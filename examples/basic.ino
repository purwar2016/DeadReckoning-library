/*
 Name:		DeadReckonerTest.ino
 Created:	5/22/2018 9:08:23 PM
 Author:	while
*/

#include <DeadReckoner.h>

// encoder pins
#define ENCODER_LEFT_PIN 2
#define ENCODER_RIGHT_PIN 3

// measurements
#define RADIUS 16 // wheel radius in mm
#define LENGTH 150 // wheel base length in mm
#define TICKS_PER_REV 900

// time intervals
#define POSITION_COMPUTE_INTERVAL 50000 // microseconds
#define SEND_INTERVAL 100000 // microseconds

// Number of left and right tick counts on the encoder.
volatile int leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;


DeadReckoner deadReckoner(&leftTicks, &rightTicks, TICKS_PER_REV, RADIUS, LENGTH);


typedef union {
	float number;
	uint8_t bytes[4];
} FLOATUNION_t;

void sendCoordinateData(double x, double y) {
	Serial.write((uint8_t)255);
	sendFloatAsBytes(x);
	sendFloatAsBytes(y);
}

void sendFloatAsBytes(float val) {
	FLOATUNION_t floatUnion;
	floatUnion.number = val;
	Serial.write((char)floatUnion.bytes[3]);
	Serial.write((char)floatUnion.bytes[2]);
	Serial.write((char)floatUnion.bytes[1]);
	Serial.write((char)floatUnion.bytes[0]);
}

void pulseLeft() { leftTicks++; }
void pulseRight() { rightTicks++; }

/**
Attaches interrupt and disables all serial communications.
This is necessary because when interrupts are active, incoming serial communication can be lost.
*/
void attachInterrupts() {
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, HIGH);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, HIGH);
}



void setup() {
	attachInterrupts();
	Serial.begin(115200);
}

// the loop function runs over and over again until power down or reset
void loop() {
	if (micros() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
		deadReckoner.integratePosition();
		prevPositionComputeTime = micros();
	}

	if (micros() - prevSendTime > SEND_INTERVAL) {

		double x = deadReckoner.getX(); // x position
		double y = deadReckoner.getY(); // y position

		double wl = deadReckoner.getWl(); // angular velocity left 
		double wr = deadReckoner.getWr(); // angular velocity right

		sendCoordinateData(x, y);


		Serial.print("x: "); Serial.print(x);
		Serial.print("\ty: "); Serial.println(y);

		Serial.print("leftTicks: "); Serial.print(leftTicks);
		Serial.print("\trightTicks: "); Serial.println(rightTicks);

		prevSendTime = micros();
	}
}
