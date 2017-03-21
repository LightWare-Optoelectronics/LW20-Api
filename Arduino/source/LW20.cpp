#include "lw20.h"
#define LW20_API_IMPLEMENTATION
#include "lw20api.h"

LW20::LW20(HardwareSerial& serial, unsigned long BaudRate)
{
	_serial = &serial;
	_serialBaud = BaudRate;
	_lw20 = lw20CreateLW20();
}

bool LW20::init()
{
	// TODO: Only init if we need it
	_serial->begin(_serialBaud);
	return runEventLoop();
}

bool LW20::_sendPacket(lwCmdPacket* Packet)
{
	_serial->write(Packet->buffer, Packet->length);
	return true;
}

bool LW20::_getPacket(lwResponsePacket* Packet)
{
	// TODO: Reset response packet function.
	Packet->data.length = 0;
	Packet->type = LWC_NONE;

	while (true)
	{
		if (_inputBufferSize == 0)
		{
			int32_t timeout = millis() + 2000;
			bool timedOut = false;
			while (!(timedOut = !(millis() < timeout)))
			{	
				int32_t readBytes = 0;
				while (readBytes < 128 && _serial->available() > 0)
				{
					_inputBuffer[readBytes++] = (uint8_t)_serial->read();
				}
				
				_inputBufferSize = readBytes;
				if (_inputBufferSize != 0)
					break;
			}

			if (timedOut)
			{
				return false;
			}
		}

		lwResolvePacketResult packetResolve = lw20ResolvePacket(Packet, _inputBuffer, _inputBufferSize);

		// If we read bytes, shift the input buffer
		if (packetResolve.bytesRead > 0)
		{
			int32_t remaining = _inputBufferSize - packetResolve.bytesRead;
			for (int i = 0; i < remaining; ++i)
				_inputBuffer[i] = _inputBuffer[packetResolve.bytesRead + i];
			
			_inputBufferSize = remaining;
		}
		
		if (packetResolve.status == LWRPS_COMPLETE)
		{
			//printf("_getPacket: resolved: %d\n", Packet->type);
			// Can intercept and handle streaming data directly here.
			// But make sure you eventually give the pump a packet, or exceed timeout.
			// Could handle streaming data directly here
			return true;
			
		}
	}

	return false;
}

bool LW20::runEventLoop(lwResponsePacket* Response)
{
	lwResponsePacket packet = {};
	packet.type = LWC_NONE;
	packet.data.length = 0;

	// TODO: Maybe convert to simple params passed into event loop?
	lwEventLoopUpdate update = {};
	update.lw20 = &_lw20;
	update.responsePacket = &packet;
	update.sendPacket.length = 0;

	bool completed = false;
	bool running = true;

	while (running)
	{
		lwEventLoopResult result = lw20PumpEventLoop(&update);
	
		switch (result)
		{
			case LWELR_INITED:
			{
			} break;

			case LWELR_SEND:
			{
				if (!_sendPacket(&update.sendPacket))
					running = false;
			} break;

			case LWELR_SLEEP:
			{
				delay(update.timeMS);
			} break;

			case LWELR_GET_PACKET:
			{
				if (!_getPacket(&packet))
					running = false;
			} break;

			case LWELR_AGAIN:
			{
			} break;

			case LWELR_ERROR:
			case LWELR_TIMEOUT:
			{
				running = false;
			} break;

			case LWELR_FEEDBACK:
			{
				if (Response)
					memcpy(Response, &packet, sizeof(lwResponsePacket));
				
				running = false;
				completed = true;
			} break;

			case LWELR_COMPLETED:
			{
				running = false;
				completed = true;
			} break;
		};
	}

	return completed;
}

bool LW20::disconnect()
{
	_serial->end();

	return true;	
}

bool LW20::setLaserParams(int ModeSpeed)
{
	// TODO: Watch this.	
	lwCmdPacket* packet = (lwCmdPacket*)&_lw20.packetBuf;

	packetClear(packet);
	packetWriteString(packet, "#lm,");
	packetWriteInt(packet, ModeSpeed);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_LASER_MODE;
	return runEventLoop();
}

bool LW20::setServoParams(float PWMMin, float PWMMax, float PWMScale)
{	
	lwCmdPacket* packet = (lwCmdPacket*)&_lw20.packetBuf;

	packetClear(packet);
	packetWriteString(packet, "#sc,0\r");
	_lw20.sendCommand = LWC_SERVO_CONNECTED;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#swl,");
	packetWriteFloat(packet, PWMMin);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_PWM_MIN;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#swh,");
	packetWriteFloat(packet, PWMMax);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_PWM_MAX;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sws,");
	packetWriteFloat(packet, PWMScale);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_PWM_SCALE;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sc,1\r");
	_lw20.sendCommand = LWC_SERVO_CONNECTED;
	if (!runEventLoop()) return false;
}

bool LW20::setScanParams(float FOVMin, float FOVMax, int32_t Steps, float Lag)
{
	lwCmdPacket* packet = (lwCmdPacket*)&_lw20.packetBuf;

	packetClear(packet);
	packetWriteString(packet, "#sfl,");
	packetWriteInt(packet, FOVMin);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_FOV_LOW;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sfh,");
	packetWriteInt(packet, FOVMax);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_FOV_HIGH;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sr,");
	packetWriteInt(packet, Steps);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_STEPS;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sl,");
	packetWriteFloat(packet, Lag);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_LAG;
	if (!runEventLoop()) return false;

	return false;
}

bool LW20::setAlarmAParams(float Distance, float FOVMin, float FOVMax)
{
	lwCmdPacket* packet = (lwCmdPacket*)&_lw20.packetBuf;

	packetClear(packet);
	packetWriteString(packet, "#laa,");
	packetWriteFloat(packet, Distance);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_LASER_ALARM_A_DISTANCE;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sal,");
	packetWriteInt(packet, FOVMin);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_ALARM_A_LOW;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sah,");
	packetWriteInt(packet, FOVMax);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_ALARM_A_HIGH;
	if (!runEventLoop()) return false;

	return false;
}

bool LW20::setAlarmBParams(float Distance, float FOVMin, float FOVMax)
{
	lwCmdPacket* packet = (lwCmdPacket*)&_lw20.packetBuf;

	packetClear(packet);
	packetWriteString(packet, "#lab,");
	packetWriteFloat(packet, Distance);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_LASER_ALARM_B_DISTANCE;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sbl,");
	packetWriteInt(packet, FOVMin);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_ALARM_B_LOW;
	if (!runEventLoop()) return false;

	packetClear(packet);
	packetWriteString(packet, "#sbh,");
	packetWriteInt(packet, FOVMax);
	packetWriteString(packet, LW20_TERMINAL);
	_lw20.sendCommand = LWC_SERVO_ALARM_B_HIGH;
	if (!runEventLoop()) return false;

	return false;
}

bool LW20::startScan()
{
	lwCmdPacket* packet = (lwCmdPacket*)&_lw20.packetBuf;

	packetClear(packet);
	packetWriteString(packet, "#ss,1\r");
	_lw20.sendCommand = LWC_SERVO_SCANNING;
	if (!runEventLoop()) return false;

	return false;
}

bool LW20::stopScan()
{
	lwCmdPacket* packet = (lwCmdPacket*)&_lw20.packetBuf;

	packetClear(packet);
	packetWriteString(packet, "#ss,0\r");
	_lw20.sendCommand = LWC_SERVO_SCANNING;
	if (!runEventLoop()) return false;

	return false;
}

bool LW20::checkAlarmStatus(bool* A, bool* B)
{
	lwCmdPacket* packet = (lwCmdPacket*)&_lw20.packetBuf;
	lwResponsePacket response = {};

	packetClear(packet);
	packetWriteString(packet, "#a\r");
	_lw20.sendCommand = LWC_ALARM_STATE_BOTH;
	if (!runEventLoop(&response)) return false;

	*A = (response.intValue & 1) != 0;
	*B = (response.intValue & 2) != 0;

	return false;
}