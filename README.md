# DeadReckoning-library

This library has been developed by Jae An for the PreciseMotion library (PREMO) for precise motion control of low-cost kit robots.

This library uses dead reckoning on a differential drive robots with encoders to estimate the position of the robot real time. The library can also be used with a gyroscope for further accuracy.

The main limiation is that this library primarily uses wheel encoder data and assumes no wheel slipage. 
This issue can be significantly mitigated by using a gyroscope, but translational data from wheel slip cannot be mitigated without an external position reference.

This library is useful in scenarious where wheel slipage is unlikely such as a robot toy moving on a hard surface.

Documentation:
https://github.com/jaean123/DeadReckoning-library/wiki/Documentation
