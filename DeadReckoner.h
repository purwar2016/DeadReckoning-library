/*
Name: ArduinoBlue.cpp
Created: 6/28/2017 11:00:39 AM
Author: Jae An
Contact: jaean37@gmail.com
*/

#ifndef _ArduinoBlue_h
#define _ArduinoBlue_h
#include <Arduino.h>
#include <LinkedList.h>

#define TEXT_SEND_TRANSMISSION 248
#define CONNECTION_CHECK 249
#define TRANSMISSION_END 250
#define DRIVE_TRANSMISSION 251
#define BUTTON_TRANSMISSION 252
#define SLIDER_TRANSMISSION 253
#define TEXT_TRANSMISSION 254
#define PATH_TRANSMISSION 255

#define DEFAULT_VALUE 255

#define TEXT_TRANSMISSION_TIMEOUT 5000 // ms
#define SHORT_TRANSMISSION_TIMEOUT 500
#define PATH_TRANSMISSION_TIMEOUT 10000

const uint8_t DEFAULT_STEERING = 49;
const uint8_t DEFAULT_THROTTLE = 49;
const uint8_t MAX_SHORT_SIGNAL_LENGTH = 3;

struct PathPoint {
	double x;
	double y;
};

class ArduinoBlue
{
public:
    ArduinoBlue(Stream &output);
    int getButton();
    int getSliderId();
    int getSliderVal();
    int getThrottle();
    int getSteering();
	LinkedList<PathPoint*> getPath();
    bool checkBluetooth();
    bool isConnected();
	bool isPathAvailable();
    void sendText(String msg);
    void sendMessage(String msg);
	static float bytesToFloat(uint8_t u1, uint8_t u2, uint8_t u3, uint8_t u4);
    String getText();
private:
    Stream & _bluetooth;
    uint8_t _signal[MAX_SHORT_SIGNAL_LENGTH];
    uint8_t _signalLength = 0;
    uint8_t _throttle = DEFAULT_STEERING;
    uint8_t _steering = DEFAULT_THROTTLE;
    uint8_t _sliderVal = DEFAULT_VALUE;
    uint8_t _sliderId = DEFAULT_VALUE;
    uint8_t _button = DEFAULT_VALUE;
	bool _pathAvailable = false;
    String _text;
	LinkedList<PathPoint*> _path = LinkedList<PathPoint*>();
    void clearSignalArray();
    void pushToSignalArray(uint8_t elem);
    void storeShortTransmission();
	void storePathTransmission();
    void processDriveTransmission();
    void processButtonTransmission();
    void processSliderTransmission();
    void processTextTransmission();
    void processPathTransmission();
    String readString();
};

#endif
