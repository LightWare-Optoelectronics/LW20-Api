#include <iostream>

#define LW20_API_IMPLEMENTATION
#include "..\\lw20api.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

// TODO: Linux & Windows usage code is very similar, only coms, time, differences

//-------------------------------------------------------------------------
// Com Port Implementation.
//-------------------------------------------------------------------------
HANDLE comHandle;

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
				//std::cout << "Bytes read: " << bytesRead << "\n";
				return bytesRead;
			}
		}
	}
	else if (bytesRead > 0)
	{
		//std::cout << "Bytes read: " << bytesRead << "\n";
		return bytesRead;
	}

	return 0;
}

//-------------------------------------------------------------------------

struct lwSensorContext
{
	lwLW20		lw20;
	HANDLE		serialPort;
	uint8_t		inputBuffer[128];
	int32_t		inputBufferSize;
};

bool sendPacket(lwLW20* Lw20, lwCmdPacket* Packet)
{
	//std::cout << "Send Packet " << Packet->length << "\n";
	lwSensorContext* context = (lwSensorContext*)Lw20->userData;
	if (serialWrite(&context->serialPort, Packet->buffer, Packet->length) != -1)
		return true;

	return false;
}

bool getPacket(lwLW20* Lw20, lwResponsePacket* Packet)
{
	//std::cout << "Get Packet\n";
	lwSensorContext* context = (lwSensorContext*)Lw20->userData;

	// TODO: Timeout here.
	while (true)
	{
		if (context->inputBufferSize == 0)
		{
			int bytesRead = 0;
			if ((bytesRead = serialRead(&context->serialPort, context->inputBuffer, sizeof(context->inputBuffer))) != -1)
				context->inputBufferSize = bytesRead;
			else
				return false;
		}

		lwResolvePacketResult packetResolve = lw20ResolvePacket(&context->lw20.response, context->inputBuffer, context->inputBufferSize);

		// NOTE: You can use a circular buffer or so to avoid the shuffle.
		if (packetResolve.bytesRead > 0)
		{
			int32_t remaining = context->inputBufferSize - packetResolve.bytesRead;
			for (int i = 0; i < remaining; ++i)
				context->inputBuffer[i] = context->inputBuffer[packetResolve.bytesRead + i];

			context->inputBufferSize = remaining;
		}
		
		if (packetResolve.status == LWRPS_COMPLETE)
		{
			//std::cout << "Got packet " << Packet->type << "\n";
			return true;
		}
	}

	return true;
}

bool sleep(lwLW20* Lw20, int32_t TimeMS)
{
	//std::cout << "Sleep " << TimeMS << "\n";
	Sleep(TimeMS);
	return true;
};

bool streamResponse(lwLW20* Lw20, lwResponsePacket* Packet)
{
	if (Packet->type == LWC_SERVO_SCAN)
	{
		std::cout << "Scan: " << Packet->scanSample.angle << " " << Packet->scanSample.firstPulse << " " << Packet->scanSample.lastPulse << "\n";
	}
	else if (Packet->type == LWC_SERVO_POSITION)
	{
		std::cout << "Pos: " << Packet->floatValue << "\n";
	}
	else if (Packet->type == LWC_LASER_TEMPERATURE)
	{
		std::cout << "Temp: " << Packet->floatValue << "\n";
	}

	return true;
};

int main()
{
	std::cout << "LW20 Api\n";

	lwSensorContext context = {};
	context.lw20 = lw20CreateLW20();
	context.lw20.userData = &context;
	serialConnect(&context.serialPort, "\\\\.\\COM8", 115200);

	lwServiceContext serviceContext = {};
	serviceContext.sendPacketCallback = sendPacket;
	serviceContext.getPacketCallback = getPacket;
	serviceContext.sleepCallback = sleep;
	serviceContext.streamCallback = streamResponse;

	// NOTE: Run event loop for first time init.
	runEventLoop(&context.lw20, &serviceContext);
	
	executeCommand(&context.lw20, &serviceContext, "?\r", LWC_PRODUCT);

	if (context.lw20.response.type == LWC_PRODUCT)
		std::cout << "Product: " << context.lw20.response.product.model << "\n";

	packetWriteLaserMode(&context.lw20.command, LWMS_48);
	runEventLoop(&context.lw20, &serviceContext);

	executeCommand(&context.lw20, &serviceContext, "$1lt\r", LWC_STREAM_1);
	executeCommand(&context.lw20, &serviceContext, "$2ss\r", LWC_STREAM_2);

	// NOTE: Stuck here infinitely.
	runEventLoop(&context.lw20, &serviceContext, true);
	
	std::cout << "Press Enter to Exit...\n";
	std::cin.ignore();
	return 0;
}