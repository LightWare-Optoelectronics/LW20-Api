
#ifndef LW20_H
#define LW20_H

#include "Arduino.h"
#include "lw20api.h"

#define LW20_MODE_388	1
#define LW20_MODE_194	2
#define LW20_MODE_129	3
#define LW20_MODE_97	4
#define LW20_MODE_77	5
#define LW20_MODE_64	6
#define LW20_MODE_55	7
#define LW20_MODE_48	8

class LW20
{
public:

	LW20(HardwareSerial& serial, unsigned long BaudRate);
	bool init();
	bool disconnect();
	bool runEventLoop(lwResponsePacket* Response = 0);

	bool setLaserParams(int ModeSpeed);
	bool setServoParams(float PWMMin, float PWMMax, float PWMScale);
	bool setScanParams(float FOVMin, float FOVMax, int32_t Steps, float Lag);
	bool setAlarmAParams(float Distance, float FOVMin, float FOVMax);
	bool setAlarmBParams(float Distance, float FOVMin, float FOVMax);

	bool startScan();
	bool stopScan();
	bool checkAlarmStatus(bool* A, bool* B);

private:

	HardwareSerial* _serial;
	unsigned long _serialBaud;

	lwLW20	_lw20;
	uint8_t	_inputBuffer[128];
	int32_t	_inputBufferSize;

	bool _sendPacket(lwCmdPacket* Packet);
	bool _getPacket(lwResponsePacket* Packet);
};

#endif