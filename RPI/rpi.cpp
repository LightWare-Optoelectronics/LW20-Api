//-------------------------------------------------------------------------
// LightWare LW20 RaspberryPI Sample
//-------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/poll.h>
#include <sys/eventfd.h>
#include <linux/input.h>
#include <stdint.h>

#define LW20_API_IMPLEMENTATION
#include "lw20api.h"

struct lwSerialPort
{
	int fd;
	bool connected;
};

struct lwSensorContext
{
	lwLW20			lw20;
	lwSerialPort 	serial;
	uint8_t			inputBuffer[128];
	int32_t			inputBufferSize;
};

//-------------------------------------------------------------------------
// Platform Specific Functions.
//-------------------------------------------------------------------------
inline int64_t PlatformGetMicrosecond()
{
	timespec time;
	clock_gettime(CLOCK_REALTIME, &time);

	return time.tv_sec * 1000000 + time.tv_nsec / 1000;
}

inline int32_t PlatformGetMS()
{
	return (PlatformGetMicrosecond() / 1000);
}

timespec timeDiff(timespec &Start, timespec &End)
{
	timespec temp;

	if ((End.tv_nsec - Start.tv_nsec) < 0)
	{
		temp.tv_sec = End.tv_sec - Start.tv_sec - 1;
		temp.tv_nsec = 1000000000 + End.tv_nsec - Start.tv_nsec;
	}
	else
	{
		temp.tv_sec = End.tv_sec - Start.tv_sec;
		temp.tv_nsec = End.tv_nsec - Start.tv_nsec;
	}

	return temp;
}

//-------------------------------------------------------------------------
// Serial Communication.
//-------------------------------------------------------------------------

bool serialClose(lwSerialPort* ComPort)
{
	if (ComPort != 0 && ComPort->fd >= 0)
	{
		close(ComPort->fd);
	}

	ComPort->fd = -1;
	ComPort->connected = false;

	return true;
}

bool serialOpen(lwSerialPort* ComPort)
{
	const char* portName = "/dev/ttyUSB0";
	serialClose(ComPort);

	int fd = -1;
	printf("Attempt com connection: %s\n", portName);
		
	fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
	
	if (fd < 0)
	{
		printf("Couldn't open serial port!\n");
		return false;
	}

	//int speed = B230400;
	int speed = B921600;
	int parity = 0;

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0)
	{
		printf("Error from tcgetattr\n");
		return false;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	// TODO: Set every possible setting without retreving tty attrs from system.
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;
	
	tty.c_iflag &= ~IGNBRK;
	tty.c_iflag &= ~ICRNL;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

	tty.c_lflag = 0;

	tty.c_oflag = 0;

	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1; // TODO: Check this 100ms wait?

	/*
	// NOTE: Check OpenHardwareSerial()
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_lflag = 0;
	tty.c_oflag = 0;

	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 5;

	tty.c_iflag &= ~IGNBRK;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	*/

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("Error from tcsetattr\n");
		return false;
	}

	printf("Connected\n");
	ComPort->fd = fd;
	ComPort->connected = true;

	return true;
}

int32_t serialRead(lwSerialPort* ComPort, char *Buffer, int32_t BufferSize)
{
	if (!ComPort)
	{
		printf("Can't read from null coms\n");
		return 0;
	}

	if (!ComPort->connected)
	{
		printf("Can't read from non connected coms\n");
		return 0;
	}

	errno = 0;
	int readBytes = read(ComPort->fd, Buffer, BufferSize);

	if (readBytes <= 0)
	{
		// TODO: what if we have a legit no data case, what is the error set to??
		// Also, what if there is more data than we read, since EAGAIN is the same error...
		printf("No Data: %d Error %d (%s)\n", readBytes, errno, strerror(errno));
		serialClose(ComPort);
		return 0;
	}

	return readBytes;
}

// NOTE: Doesn't close port if we have an error
int32_t serialReadNoError(lwSerialPort* ComPort, char *Buffer, int32_t BufferSize)
{
	if (!ComPort)
	{
		printf("Can't read from null coms\n");
		return 0;
	}

	if (!ComPort->connected)
	{
		printf("Can't read from non connected coms\n");
		return 0;
	}

	errno = 0;
	int readBytes = read(ComPort->fd, Buffer, BufferSize);

	//if (readBytes == 0)
		//printf("No Data: %d Error %d (%s)\n", readBytes, errno, strerror(errno));

	return readBytes;
}

void serialWrite(lwSerialPort* ComPort, char *Buffer, int32_t BufferSize)
{
	if (!ComPort)
	{
		printf("Can't write to null coms\n");
		return;
	}

	if (!ComPort->connected)
	{
		printf("Can't write to non connected coms\n");
		return;
	}

	int writtenBytes = write(ComPort->fd, Buffer, BufferSize);

	if (writtenBytes != BufferSize)
	{
		printf("Could not send all bytes!\n");
		// Disconnect error out here?
	}
}

bool PlatformComIsConnected(lwSerialPort* ComPort)
{
	return ComPort->connected;
}

//-------------------------------------------------------------------------
// Threading.
//-------------------------------------------------------------------------
pthread_t			gIOThread;
pthread_mutex_t		gPointMutex;
volatile int		gPointBuffer[4096];
volatile int		gPointCount = 0;
volatile int		gPointIndex = 0;

void* RunIOThread(void* Args)
{
	printf("IO Thread Running\n");

	/*
	int serialFD = OpenHardwareSerial();
	if (serialFD < 0)
	{
		printf("Invalid Serial Port\n");
		pthread_exit(0);
	}

	struct pollfd ufds[32];
	int fdCount = 1;
	
	ufds[0].fd = serialFD;
	ufds[0].events = POLLIN;

	while (true)
	{
		int rv = poll(ufds, fdCount, -1);

		if (rv == -1)
		{
			printf("poll error\n");
		}
		else if (rv == 0)
		{
			printf("poll timeout\n");
		}
		else
		{
			if (ufds[0].revents & POLLIN)
			{
				char buffer[4096];
				int readBytes = read(serialFD, buffer, 4096);				

				printf("serial %d \n", readBytes);

			}
		}
	}
	*/

	pthread_exit(0);
}

void PlatformStartThread()
{
	pthread_mutex_init(&gPointMutex, 0);
	pthread_create(&gIOThread, 0, RunIOThread, 0);
}

// TODO: Pass in timeout.
bool getPacket(lwSensorContext* Context, lwResponsePacket* Packet)
{
	uint8_t* Buffer = Context->inputBuffer;
	int32_t* BufferSize = &Context->inputBufferSize;
	
	// TODO: Reset response packet function.
	Packet->data.length = 0;
	Packet->type = LWC_NONE;

	while (true)
	{
		if (*BufferSize == 0)
		{
			int32_t timeout = PlatformGetMS() + 2000;
			bool timedOut = false;
			while (!(timedOut = !(PlatformGetMS() < timeout)))
			{	
				*BufferSize = serialReadNoError(&Context->serial, (char*)Buffer, 128);
				if (*BufferSize == -1)
				{
					printf("LWELR_GET_PACKET: Read error: %d\n", *BufferSize);
					return false;
				}
				else if (*BufferSize != 0)
				{
					printf("(%d) [\n", *BufferSize);
					for (int i = 0; i < *BufferSize; ++i)							
						if (Buffer[i] != '\n' && Buffer[i] != '\r')
							printf("%c", Buffer[i]);
					printf("]\n");

					break;
				}
				else
				{
					printf("(0)\n");
				}
			}

			if (timedOut)
			{
				printf("LWELR_GET_PACKET: Timeout\n");
				return false;
			}
		}
		
		lwResolvePacketResult packetResolve = lw20ResolvePacket(Packet, Buffer, *BufferSize);

		// If we read bytes, shift the input buffer
		if (packetResolve.bytesRead > 0)
		{
			int32_t remaining = *BufferSize - packetResolve.bytesRead;
			for (int i = 0; i < remaining; ++i)
				Buffer[i] = Buffer[packetResolve.bytesRead + i];
			
			*BufferSize = remaining;
		}

		if (packetResolve.status == LWRPS_COMPLETE)
		{
			printf("LWELR_GET_PACKET: resolved: %d\n", Packet->type);
			// Can intercept and handle streaming data directly here.
			// But make sure you eventually give the pump a packet, or exceed timeout.
			// Could handle streaming data directly here
			return true;
		}
	}

	return false;
}

bool sendPacket(lwSensorContext* Context, lwCmdPacket* Packet)
{
	printf("LWELR_SEND: (%d) [", Packet->length);
	for (int i = 0; i < Packet->length; ++i)
	{
		if (Packet->buffer[i] != '\n' && Packet->buffer[i] != '\r')
			printf("%c", Packet->buffer[i]);
	}
	printf("]\n");
	serialWrite(&Context->serial, (char*)Packet->buffer, Packet->length);

	return true;
}

// callbacks for feedback, IO, etc. Run to completion.
void runEventLoop(lwSensorContext* Context)
{
	lwResponsePacket packet = {};

	// TODO: Maybe convert to simple params passed into event loop?
	lwEventLoopUpdate update = {};
	update.lw20 = &Context->lw20;
	update.responsePacket = &packet;

	bool running = true;
	while (running)
	{
		lwEventLoopResult result = lw20PumpEventLoop(&update);
		
		switch (result)
		{
			case LWELR_SEND:
			{
				if (!sendPacket(Context, &update.sendPacket))
				{
					printf("LWELR_SEND: Error\n");
					running = false;
				}
			} break;

			case LWELR_SLEEP:
			{
				printf("LWELR_SLEEP: %dms\n", update.timeMS);
				usleep(update.timeMS * 1000);
			} break;

			case LWELR_GET_PACKET:
			{
				printf("LWELR_GET_PACKET\n");
				if (!getPacket(Context, &packet))
				{
					printf("LWELR_GET_PACKET: Error\n");
					running = false;
				}
			} break;

			case LWELR_INITED:
			{
				printf("LWELR_INITED\n");
			} break;

			case LWELR_AGAIN:
			{
			} break;

			case LWELR_ERROR:
			case LWELR_TIMEOUT:
			{
				running = false;
				printf("LWELR_TIMEOUT/ERROR\n");
			} break;

			case LWELR_FEEDBACK:
			{
				running = false;
				printf("LWELR_FEEDBACK\n");
			} break;

			case LWELR_COMPLETED:
			{
				running = false;
				printf("LWELR_COMPLETED\n");
			} break;
		};
	}
}

//-------------------------------------------------------------------------
// Application Entry.
//-------------------------------------------------------------------------
int main(int args, char **argv)
{
	printf("Program Start\n");

	timespec timeLastFrame;
	clock_gettime(CLOCK_REALTIME, &timeLastFrame);

	lwSensorContext context = {};
	context.lw20 = lw20CreateLW20();
	serialOpen(&context.serial);

	// Run the event loop to completion to init the unit.
	runEventLoop(&context);
	
	printf("Done event pump\n");

	lwLW20* lw20 = &context.lw20;
	printf("Product: %s, %f, %f\n", lw20->product.model, lw20->product.firmwareVersion, lw20->product.softwareVersion);
	
	/*
	while (1)
	{	
		timespec timeCurrentFrame;
		clock_gettime(CLOCK_REALTIME, &timeCurrentFrame);
		timespec temp = timeDiff(timeLastFrame, timeCurrentFrame);
		int64_t timeElapsed = temp.tv_sec * 1000000 + temp.tv_nsec / 1000;
		clock_gettime(CLOCK_REALTIME, &timeLastFrame);
		float elapsedTime = (float)((double)timeElapsed / 1000000.0);

		int64_t t1 = PlatformGetMicrosecond();
		//printf("Frame Time: %fsec\n", elapsedTime);
		int64_t t2 = PlatformGetMicrosecond();

		printf("Distance: %f - %dus\n", distance, (int)((t2 - t1)));
		printf("Temp: %f\n", internalTemp);
		printf("Noise: %f\n", backgroundNoise);
		printf("Strength: %d\n", signalStrength);
	}
	*/

	printf("Program End\n");
	serialClose(&context.serial);

	return 0;
}