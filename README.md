# DeadReckoning-library
This library uses dead reckoning on a differential drive Arduino robot with encoders to estimate the position of the robot real time.

The main limiation is that this library only uses wheel encoder data and assumes no wheel slipage. 
It is useful in scenarious where wheel slipage is unlikely such as a robot toy moving on a hard surface.

# Classes and Methods
### DeadReckoner class
__```DeadReckoner(volatile int *leftTicks, volatile int *rightTicks, int ticksPerRev, double radius, double length)```__
* leftTicks: Pointer to total number of encoder ticks for left wheel.
* rightTicks: Pointer to total number of encoder ticks for right wheel.
* ticksPerRev: Number of encoder ticks per revolution of wheel.
* radius: Radius of the wheel.
* length: length from the center of the left wheel to the right wheel.

__```computePosition()```__ - Compute the estimates of angular velocity of wheels and position of robot.  
__```getX()```__ - Returns the x coordinate of the latest position estimate.  
__```getY()```__ - Returns the y coordinate of the latest position estimate.  
__```getWl()```__ - Returns the latest angular velocity of the left wheel.  
__```getWr()```__ - Returns the latest angular velocity of the right wheel.  
__```getTheta()```__ - Returns the latest position angle estimate. Theta is measured from the x-axis to the center of the robot.  
Theta is  taken to be zero before the robot starts moving. 
