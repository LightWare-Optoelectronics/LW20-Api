#include "LW20.h"
#define LW20_API_IMPLEMENTATION
#include "lw20api.h"

bool sendPacket(lwLW20* Lw20, lwCmdPacket* Packet)
{
	LW20* lw20 = (LW20*)Lw20->userData;
	lw20->serial->write(Packet->buffer, Packet->length);
	return true;
}

bool getPacket(lwLW20* Lw20, lwResponsePacket* Packet)
{
	LW20* lw20 = (LW20*)Lw20->userData;
	Packet->data.length = 0;
	Packet->type = LWC_NONE;

	int32_t timeout = millis() + 2000;
	bool timedOut = false;
	while (!(timedOut = !(millis() < timeout)))
	{
		if (lw20->serial->available() > 0)
		{
			uint8_t data = (uint8_t)lw20->serial->read();
			lwResolvePacketResult packetResolve = lw20ResolvePacket(Packet, &data, 1);
		
			if (packetResolve.status == LWRPS_COMPLETE)
				return true;
		}
	}

  Serial.println("Packet Timeout");
	return false;
}

bool sleep(lwLW20* Lw20, int32_t TimeMS)
{
	delay(TimeMS);
	return true;
}

LW20::LW20(HardwareSerial& Serial, unsigned long BaudRate)
{
	serial = &Serial;
	serialBaud = BaudRate;
	lw20 = lw20CreateLW20();
	lw20.userData = this;

	serviceContext.sendPacketCallback = sendPacket;
	serviceContext.getPacketCallback = getPacket;
	serviceContext.sleepCallback = sleep;
	serviceContext.streamCallback = NULL;
}

bool LW20::init()
{
	serial->begin(serialBaud);
	return runEventLoop(&lw20, &this->serviceContext);
}

void LW20::disconnect()
{
	serial->end();
}

char* LW20::getProductName()
{
	return lw20.product.model;
}

float LW20::getFirmwareVersion()
{
	return lw20.product.firmwareVersion;
}

float LW20::getSoftwareVersion()
{
	return lw20.product.softwareVersion;
}

void LW20::setLaserParams(lwModeSpeed ModeSpeed)
{
	executeCmd_SetLaserMode(&lw20, &serviceContext, ModeSpeed);
}

void LW20::setServoParams(float PwmMin, float PwmMax, float PwmScale)
{	
	executeCmd_SetServoConnected(&lw20, &serviceContext, false);
	executeCmd_SetServoPwmMin(&lw20, &serviceContext, PwmMin);
	executeCmd_SetServoPwmMax(&lw20, &serviceContext, PwmMax);
	executeCmd_SetServoPwmScale(&lw20, &serviceContext, PwmScale);
	executeCmd_SetServoConnected(&lw20, &serviceContext, true);
}

void LW20::setScanParams(float FovMin, float FovMax, int32_t Steps, float LagAngle)
{
	executeCmd_SetServoFovLow(&lw20, &serviceContext, FovMin);
	executeCmd_SetServoFovHigh(&lw20, &serviceContext, FovMax);
	executeCmd_SetServoSteps(&lw20, &serviceContext, Steps);
	executeCmd_SetServoLag(&lw20, &serviceContext, LagAngle);
}

void LW20::setAlarmAParams(float Distance, float FovMin, float FovMax)
{
	executeCmd_SetLaserAlarmA(&lw20, &serviceContext, Distance);
	executeCmd_SetServoAlarmALow(&lw20, &serviceContext, FovMin);
	executeCmd_SetServoAlarmAHigh(&lw20, &serviceContext, FovMax);
}

void LW20::setAlarmBParams(float Distance, float FovMin, float FovMax)
{
	executeCmd_SetLaserAlarmB(&lw20, &serviceContext, Distance);
	executeCmd_SetServoAlarmBLow(&lw20, &serviceContext, FovMin);
	executeCmd_SetServoAlarmBHigh(&lw20, &serviceContext, FovMax);
}

void LW20::startScan()
{
	executeCmd_SetServoScanning(&lw20, &serviceContext, true);
}

void LW20::stopScan()
{
	executeCmd_SetServoScanning(&lw20, &serviceContext, false);
}

float LW20::getDistance()
{
	return executeCmd_GetLaserDistanceFirst(&lw20, &serviceContext);
}

float LW20::getLaserDistanceFirst()
{
	return executeCmd_GetLaserDistanceFirst(&lw20, &serviceContext);
}

float LW20::getLaserDistanceLast()
{
	return executeCmd_GetLaserDistanceLast(&lw20, &serviceContext);
}

float LW20::getLaserDistance(lwPulseType Pulse, lwReturnFilter Filter)
{
	return executeCmd_GetLaserDistance(&lw20, &serviceContext, Pulse, Filter);
}

int LW20::getLaserSignalStrengthFirst()
{
	return executeCmd_GetLaserSignalStrengthFirst(&lw20, &serviceContext);
}

int LW20::getLaserSignalStrengthLast()
{
	return executeCmd_GetLaserSignalStrengthLast(&lw20, &serviceContext);
}

float LW20::getLaserOffset()
{
	return executeCmd_GetLaserOffset(&lw20, &serviceContext);
}

void LW20::setLaserOffset(float Offset)
{
	executeCmd_SetLaserOffset(&lw20, &serviceContext, Offset);
}

float LW20::getLaserAlarmA()
{
	return executeCmd_GetLaserAlarmA(&lw20, &serviceContext);
}

void LW20::setLaserAlarmA(float Distance)
{
	executeCmd_SetLaserAlarmA(&lw20, &serviceContext, Distance);
}

float LW20::getLaserAlarmB()
{
	return executeCmd_GetLaserAlarmB(&lw20, &serviceContext);
}

void LW20::setLaserAlarmB(float Distance)
{
	executeCmd_SetLaserAlarmB(&lw20, &serviceContext, Distance);
}

float LW20::getLaserAlarmHysteresis()
{
	return executeCmd_GetLaserAlarmHysteresis(&lw20, &serviceContext);
}

void LW20::setLaserAlarmHysteresis(float Distance)
{
	executeCmd_SetLaserAlarmHysteresis(&lw20, &serviceContext, Distance);
}

lwModeSpeed LW20::getLaserMode()
{
	return executeCmd_GetLaserMode(&lw20, &serviceContext);
}

void LW20::setLaserMode(lwModeSpeed Mode)
{
	executeCmd_SetLaserMode(&lw20, &serviceContext, Mode);
}

bool LW20::getLaserFiring()
{
	return executeCmd_GetLaserFiring(&lw20, &serviceContext);
}

void LW20::setLaserFiring(bool Firing)
{
	executeCmd_SetLaserFiring(&lw20, &serviceContext, Firing);
}

float LW20::getLaserNoise()
{
	return executeCmd_GetLaserNoise(&lw20, &serviceContext);
}

float LW20::getLaserTemperature()
{
	return executeCmd_GetLaserTemperature(&lw20, &serviceContext);
}

lwEncodingPattern LW20::getLaserEncoding()
{
	return executeCmd_GetLaserEncoding(&lw20, &serviceContext);
}

void LW20::setLaserEncoding(lwEncodingPattern Encoding)
{
	executeCmd_SetLaserEncoding(&lw20, &serviceContext, Encoding);
}

int LW20::getLaserLostConfirmations()
{
	return executeCmd_GetLaserLostConfirmations(&lw20, &serviceContext);
}

void LW20::setLaserLostConfirmations(int Confirmations)
{
	executeCmd_SetLaserLostConfirmations(&lw20, &serviceContext, Confirmations);
}

float LW20::getLaserGain()
{
	return executeCmd_GetLaserGain(&lw20, &serviceContext);
}

void LW20::setLaserGain(float Gain)
{
	return executeCmd_SetLaserGain(&lw20, &serviceContext, Gain);
}

bool LW20::getServoConnected()
{
	return executeCmd_GetServoConnected(&lw20, &serviceContext);
}

void LW20::setServoConnected(bool Connected)
{
	executeCmd_SetServoConnected(&lw20, &serviceContext, Connected);
}

bool LW20::getServoScanning()
{
	return executeCmd_GetServoScanning(&lw20, &serviceContext);
}

void LW20::setServoScanning(bool Scanning)
{
	executeCmd_SetServoScanning(&lw20, &serviceContext, Scanning);
}

float LW20::getServoPosition()
{
	return executeCmd_GetServoPosition(&lw20, &serviceContext);
}

void LW20::setServoPosition(float Position)
{
	executeCmd_SetServoPosition(&lw20, &serviceContext, Position);
}

float LW20::getServoPwmMin()
{
	return executeCmd_GetServoPwmMin(&lw20, &serviceContext);
}

void LW20::setServoPwmMin(float Time)
{
	executeCmd_SetServoPwmMin(&lw20, &serviceContext, Time);
}

float LW20::getServoPwmMax()
{
	return executeCmd_GetServoPwmMax(&lw20, &serviceContext);
}

void LW20::setServoPwmMax(float Time)
{
	executeCmd_SetServoPwmMax(&lw20, &serviceContext, Time);
}

float LW20::getServoPwmScale()
{
	return executeCmd_GetServoPwmScale(&lw20, &serviceContext);
}

void LW20::setServoPwmScale(float Scale)
{
	executeCmd_SetServoPwmScale(&lw20, &serviceContext, Scale);
}

lwScanType LW20::getServoScanType()
{
	return executeCmd_GetServoScanType(&lw20, &serviceContext);
}

void LW20::setServoScanType(lwScanType Type)
{
	executeCmd_SetServoScanType(&lw20, &serviceContext, Type);
}

int LW20::getServoSteps()
{
	return executeCmd_GetServoSteps(&lw20, &serviceContext);
}

void LW20::setServoSteps(int Steps)
{
	executeCmd_SetServoSteps(&lw20, &serviceContext, Steps);
}

float LW20::getServoLag()
{
	return executeCmd_GetServoLag(&lw20, &serviceContext);
}

void LW20::setServoLag(float Angle)
{
	executeCmd_SetServoLag(&lw20, &serviceContext, Angle);
}

float LW20::getServoFovLow()
{
	return executeCmd_GetServoFovLow(&lw20, &serviceContext);
}

void LW20::setServoFovLow(float Angle)
{
	executeCmd_SetServoFovLow(&lw20, &serviceContext, Angle);
}

float LW20::getServoFovHigh()
{
	return executeCmd_GetServoFovHigh(&lw20, &serviceContext);
}

void LW20::setServoFovHigh(float Angle)
{
	executeCmd_SetServoFovHigh(&lw20, &serviceContext, Angle);
}

float LW20::getServoAlarmALow()
{
	return executeCmd_GetServoAlarmALow(&lw20, &serviceContext);
}

void LW20::setServoAlarmALow(float Angle)
{
	executeCmd_SetServoAlarmALow(&lw20, &serviceContext, Angle);
}

float LW20::getServoAlarmAHigh()
{
	return executeCmd_GetServoAlarmAHigh(&lw20, &serviceContext);
}

void LW20::setServoAlarmAHigh(float Angle)
{
	executeCmd_SetServoAlarmAHigh(&lw20, &serviceContext, Angle);
}

float LW20::getServoAlarmBLow()
{
	return executeCmd_GetServoAlarmBLow(&lw20, &serviceContext);
}

void LW20::setServoAlarmBLow(float Angle)
{
	executeCmd_SetServoAlarmBLow(&lw20, &serviceContext, Angle);
}

float LW20::getServoAlarmBHigh()
{
	return executeCmd_GetServoAlarmBHigh(&lw20, &serviceContext);
}

void LW20::setServoAlarmBHigh(float Angle)
{
	executeCmd_SetServoAlarmBHigh(&lw20, &serviceContext, Angle);
}

lwAlarmState LW20::getAlarmStateBoth()
{
	return executeCmd_GetAlarmStateBoth(&lw20, &serviceContext);
}

bool LW20::getAlarmStateA()
{
	return executeCmd_GetAlarmStateA(&lw20, &serviceContext);
}

bool LW20::getAlarmStateB()
{
	return executeCmd_GetAlarmStateB(&lw20, &serviceContext);
}

lwBaudRate LW20::getComsBaudRate()
{
	return executeCmd_GetComsBaudRate(&lw20, &serviceContext);
}

void LW20::setComsBaudRate(lwBaudRate BaudRate)
{
	executeCmd_SetComsBaudRate(&lw20, &serviceContext, BaudRate);
}

int LW20::getComsAddress()
{
	return executeCmd_GetComsAddress(&lw20, &serviceContext);
}

void LW20::setComsAddress(int Address)
{
	executeCmd_SetComsAddress(&lw20, &serviceContext, Address);
}

bool LW20::getEnergyPower()
{
	return executeCmd_GetEneryPower(&lw20, &serviceContext);
}

int LW20::setEnergyPower(bool Power)
{
	executeCmd_SetEnergyPower(&lw20, &serviceContext, Power);
}
