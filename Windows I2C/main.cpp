//-------------------------------------------------------------------------
// LightWare LW20 Windows I2C Example
//-------------------------------------------------------------------------

#include <iostream>

#define LW20_API_IMPLEMENTATION
#include "..\\lw20api.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

struct lwSensorContext
{
	lwLW20		lw20;
	HANDLE		serialPort;
};

//-------------------------------------------------------------------------
// Com Port Implementation.
//-------------------------------------------------------------------------
void serialDisconnect(HANDLE* Handle)
{
	if (*Handle != INVALID_HANDLE_VALUE)
		CloseHandle(*Handle);

	*Handle = INVALID_HANDLE_VALUE;
	std::cout << "Serial disconnected\n";
}

bool serialConnect(HANDLE* Handle, const char* ComPortName, int BuadRate)
{
	*Handle = INVALID_HANDLE_VALUE;
	HANDLE handle = CreateFile(ComPortName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

	if (handle == INVALID_HANDLE_VALUE)
	{
		std::cout << "Serial Connect: Failed to open.\n";
		return false;
	}

	PurgeComm(handle, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
		
	DCB comParams = {};
	comParams.DCBlength = sizeof(comParams);
	GetCommState(handle, &comParams);
	comParams.BaudRate = BuadRate;
	comParams.ByteSize = 8;
	comParams.StopBits = ONESTOPBIT;
	comParams.Parity = NOPARITY;
	comParams.fDtrControl = DTR_CONTROL_ENABLE;
	comParams.fRtsControl = DTR_CONTROL_ENABLE;

	BOOL status = SetCommState(handle, &comParams);

	if (status == FALSE)
	{
		// NOTE: Some poorly written USB<->Serial drivers require the state to be set twice.
		std::cout << "Serial Connect: Setting Comm State twice." << GetLastError() << "\n";
		status = SetCommState(handle, &comParams);

		if (status == FALSE)
		{
			std::cout << "Serial Connect: Failed to set comm state." << GetLastError() << "\n";
			serialDisconnect(&handle);
			return false;
		}
	}

	COMMTIMEOUTS timeouts = {};
	GetCommTimeouts(handle, &timeouts);
	timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 10;
	//timeouts.WriteTotalTimeoutConstant = 0;
	//timeouts.WriteTotalTimeoutMultiplier = 0;
	SetCommTimeouts(handle, &timeouts);

	*Handle = handle;

	std::cout << "Serial Connect: Connected to " << ComPortName << ".\n";

	return true;
}

int serialWrite(HANDLE* Handle, uint8_t* Data, int32_t Length)
{
	if (*Handle == INVALID_HANDLE_VALUE)
	{
		std::cout << "Serial Write: Invalid Serial Port.\n";
		return -1;
	}

	OVERLAPPED overlapped = {};
	DWORD bytesWritten = 0;

	if (!WriteFile(*Handle, Data, Length, &bytesWritten, &overlapped))
	{
		if (GetLastError() != ERROR_IO_PENDING)
		{
			std::cout << "Serial Write: Write Failed.\n";
			return -1;
		}
		else
		{
			if (!GetOverlappedResult(*Handle, &overlapped, &bytesWritten, true))
			{
				std::cout << "Serial Write: Waiting Error.\n";
				return -1;
			}
			else
			{
				if (bytesWritten != Length)
					return -1;

				return bytesWritten;
			}
		}
	}
	else
	{
		if (bytesWritten != Length)
			return -1;

		return bytesWritten;
	}

	return -1;
}

int serialRead(HANDLE* Handle, uint8_t* Buffer, int32_t BufferSize)
{
	OVERLAPPED overlapped = {};
	DWORD bytesRead;

	if (!ReadFile(*Handle, Buffer, BufferSize, &bytesRead, &overlapped))
	{
		if (GetLastError() != ERROR_IO_PENDING)
		{
			std::cout << "Serial Read: IO Pending Error.\n";
			return -1;
		}
		else
		{
			if (!GetOverlappedResult(*Handle, &overlapped, &bytesRead, true))
			{
				std::cout << "Serial Read: Waiting Error.\n";
				return -1;
			}
			else if (bytesRead > 0)
			{
				return bytesRead;
			}
		}
	}
	else if (bytesRead > 0)
	{
		return bytesRead;
	}

	return 0;
}

//-------------------------------------------------------------------------

bool sendPacketI2C(lwLW20* Lw20, lwCmdPacket* Packet)
{
	uint8_t data[256];

	data[0] = 0x54;
	data[1] = 0xCC;
	data[2] = Packet->length;
	memcpy(data + 3, Packet->buffer, Packet->length);

	lwSensorContext* context = (lwSensorContext*)Lw20->userData;
	if (serialWrite(&context->serialPort, data, Packet->length + 3) != -1)
	{
		uint8_t recvBuf;
		int result = 0;
		while ((result = serialRead(&context->serialPort, &recvBuf, 1)) == 0);

		if (result == -1)
			return false;

		return true;
	}

	return false;
}

bool getPacketI2C(lwLW20* Lw20, lwResponsePacket* Packet)
{
	lwSensorContext* context = (lwSensorContext*)Lw20->userData;

	uint8_t data[] = { 0x54, 0xCD, 32 };

	while (true)
	{
		if (serialWrite(&context->serialPort, data, 3) == -1)
			return false;

		int bytesRead = 0;
		uint8_t recvBuf[33] = {};
		while ((bytesRead = serialRead(&context->serialPort, recvBuf, 32)) == 0);

		if (bytesRead == -1)
		{
			std::cout << "Read error\n";
			return false;
		}
		else if (bytesRead == 32)
		{
			if (recvBuf[0] != 0)
			{
				for (int i = 0; i < 33; ++i)
				{
					if (recvBuf[i] == 0)
					{
						recvBuf[i] = '\r';
						break;
					}
				}

				context->lw20.response.data.length = 0;
				lwResolvePacketResult packetResolve = lw20ResolvePacket(&context->lw20.response, recvBuf, 33);

				if (packetResolve.status == LWRPS_COMPLETE)
				{
					return true;
				}
				else
				{
					context->lw20.response.data.length = 0;
					std::cout << "Packet Error\n";
					return false;
				}
			}
		}
		else
		{
			std::cout << "Expected 32 bytes in single read\n";
			return false;
		}
	}

	return false;
}

bool sleep(lwLW20* Lw20, int32_t TimeMS)
{
	Sleep(TimeMS);
	return true;
};

//-------------------------------------------------------------------------
// Application Entry.
//-------------------------------------------------------------------------
int main()
{
	std::cout << "LW20 Api\n";

	lwSensorContext context = {};
	context.lw20 = lw20CreateLW20();
	context.lw20.userData = &context;
	serialConnect(&context.serialPort, "\\\\.\\COM3", 115200);

	lwServiceContext serviceContext = {};
	serviceContext.sendPacketCallback = sendPacketI2C;
	serviceContext.getPacketCallback = getPacketI2C;
	serviceContext.sleepCallback = sleep;
	serviceContext.streamCallback = NULL;

	// NOTE: Run event loop for first time init.
	runEventLoop(&context.lw20, &serviceContext);
	
	executeCmd_GetProduct(&context.lw20, &serviceContext);

	if (context.lw20.response.type == LWC_PRODUCT)
		std::cout << "Product: " << context.lw20.response.product.model << "\n";

	packetWrite_SetLaserMode(&context.lw20.command, LWMS_48);
	runEventLoop(&context.lw20, &serviceContext);

	while (true)
	{
		packetWrite_GetLaserDistanceFirst(&context.lw20.command);
		runEventLoop(&context.lw20, &serviceContext);

		if (context.lw20.response.type == LWC_LASER_DISTANCE_FIRST)
			std::cout << "Distance: " << context.lw20.response.floatValue << "\n";
	}

	serialDisconnect(&context.serialPort);
	
	std::cout << "Press Enter to Exit...\n";
	std::cin.ignore();
	return 0;
}