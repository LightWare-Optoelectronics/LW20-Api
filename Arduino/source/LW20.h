#ifndef LW20_H
#define LW20_H

#include "Arduino.h"
#include "lw20api.h"

class LW20
{
public:

	lwLW20 lw20;
	lwServiceContext serviceContext;
	HardwareSerial* serial;
	unsigned long serialBaud;
	
	LW20(HardwareSerial& Serial, unsigned long BaudRate);
	bool init();
	void disconnect();

	// Product Information
	char* getProductName();
	float getFirmwareVersion();
	float getSoftwareVersion();

	// Helper Commands
	void setLaserParams(lwModeSpeed ModeSpeed);
	void setServoParams(float PWMMin, float PWMMax, float PWMScale);
	void setScanParams(float FOVMin, float FOVMax, int32_t Steps, float Lag);
	void setAlarmAParams(float Distance, float FOVMin, float FOVMax);
	void setAlarmBParams(float Distance, float FOVMin, float FOVMax);	
	void startScan();
	void stopScan();
	float getDistance();

	// Commands
	float getLaserDistanceFirst();	
	float getLaserDistanceLast();	
	float getLaserDistance(lwPulseType Pulse, lwReturnFilter Filter);
	
	float getLaserSignalStrengthFirst();	
	float getLaserSignalStrengthLast();
	
	float getLaserOffset();
	void setLaserOffset(float Offset);

	float getLaserAlarmA();
	void setLaserAlarmA(float Distance);
	float getLaserAlarmB();
	void setLaserAlarmB(float Distance);
	float getLaserAlarmHysteresis();
	void setLaserAlarmHysteresis(float Distance);

	lwModeSpeed getLaserMode();
	void setLaserMode(lwModeSpeed Mode);

	bool getLaserFiring();
	void setLaserFiring(bool Firing);

	float getLaserNoise();

	float getLaserTemperature();

	lwEncodingPattern getLaserEncoding();
	void setLaserEncoding(lwEncodingPattern Encoding);
	
	int getLaserLostConfirmations();
	void setLaserLostConfirmations(int Confirmations);
	
	float getLaserGain();
	void setLaserGain(float Gain);

	bool getServoConnected();
	void setServoConnected(bool Connected);

	bool getServoScanning();
	void setServoScanning(bool Scanning);

	float getServoPosition();
	void setServoPosition(float Position);

	float getServoPwmMin();
	void setServoPwmMin(float Time);

	float getServoPwmMax();
	void setServoPwmMax(float Time);

	float getServoPwmScale();
	void setServoPwmScale(float Scale);

	lwScanType getServoScanType();
	void setServoScanType(lwScanType Type);

	int getServoSteps();
	void setServoSteps(int Steps);

	float getServoLag();
	void setServoLag(float Angle);

	float getServoFovLow();
	void setServoFovLow(float Angle);

	float getServoFovHigh();
	void setServoFovHigh(float Angle);

	float getServoAlarmALow();
	void setServoAlarmALow(float Angle);

	float getServoAlarmAHigh();
	void setServoAlarmAHigh(float Angle);

	float getServoAlarmBLow();
	void setServoAlarmBLow(float Angle);

	float getServoAlarmBHigh();
	void setServoAlarmBHigh(float Angle);	

	lwAlarmState getAlarmStateBoth();
	bool getAlarmStateA();
	bool getAlarmStateB();

	lwBaudRate getComsBaudRate();
	void setComsBaudRate(lwBaudRate BaudRate);

	int getComsAddress();
	void setComsAddress(int Address);

	bool getEnergyPower();
	int setEnergyPower(bool Power);
};

#endif
