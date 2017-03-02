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

struct lwSerialPort
{
	int fd;
	bool connected;
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
// API Coms Implementation.
//-------------------------------------------------------------------------

lwSerialPort _globalSerialPort;

void serialWrite(lwSerialPort* ComPort, char *Buffer, int32_t BufferSize);
int32_t serialReadNoError(lwSerialPort* ComPort, char *Buffer, int32_t BufferSize);

int32_t lw20_comsWrite(uint8_t* Buffer, int32_t Size)
{
	serialWrite(&_globalSerialPort, (char*)Buffer, Size);
	return 0;
}

int32_t lw20_comsRead(uint8_t* Buffer, int32_t Size)
{
	// TODO: Handle read error.
	int32_t result = serialReadNoError(&_globalSerialPort, (char*)Buffer, Size);
	return result;
}

int32_t lw20_getMillis()
{
	return (PlatformGetMicrosecond() / 1000);
}

void lw20_sleep(int32_t Milliseconds)
{
	usleep(Milliseconds * 1000);
}

#define LW20_API_IMPLEMENTATION
#define DEBUG_PRINT(A) printf(A)
#include "lw20api.h"

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

//-------------------------------------------------------------------------
// Layered architectural approach.
//-------------------------------------------------------------------------
/*

Overview:
The layered structure of the API opens a variety of options for integration within your own application framework.
You can select a single layer to work with, or use components from each that accomplish your goals. Note: Not all
components can be mixed, or can they?

Layer 1: Event System & Primary command interface. Entirely non-blocking.
Event system unrelies all lw20 activity.

Events:
- wait for data
- sleep for
- command feedback result
- send packet
- pump again.

The event system requires continuous pumping that can fit within various overall architectural situations.
Single/multi threaded.

Layer 2: Auto pump commands. Blocking in most situations? No allocations
By passing callbacks to your IO functions this layer will automatically manage the event loops.
Most auto-pump commands will wait until their command buffers have completed. Semi-pumping with timeouts?

Layer 2.5: Need an extension on the auto-pump situations? No allocations
Maybe just some extended auto-pump commands will be fine.

Layer 3: Communication, Threading & Command Buffers. (UDP/TCP/I2C/Serial) Entirely blocking? Allocations
This layer will handle all aspects of communication over various protocols & interfaces.
Usually for rapid prototyping. Most applications already have some internal communication structure in
place that they wish to use. Threading to allow for stream data capturing?

Layer 1:

// Every command requires an event pump cycle to be managed to completion.
// If you have streaming commands, they can also be sent through. Otherwise they are ignored.
// You should hit the pump until you get COMPLETED.

init();

setExecutingCommand(cmd, retries, timeouts, response, allow stream);

while(true)
{
	result = pump(io, timeout); // Do we really need the timeout here? The pump never blocks.

	if (result == SEND)
	if (result == SLEEP)
	if (result == IO_FLUSH) // do we really need to flush io?
	if (result == RESULT)
	if (result == AGAIN)
	if (result == IO_WAIT)
		recvData();
	if (result == ERROR)
		timeout
		other failure
	if (result == TIMEOUT)
	if (result == COMPLETED)
		done
	if (result == INIT_SUCCESS)	
}

Layer 2:

init();
// single command inits, config set that doesn't implement command buffers, because allocations.

// Config block
lw20LaserConf laser = lw20CreateDefaultConfig();
conf.baud = 921600;
conf.mode = 1;
conf.offset = 0;
conf.alarmA = 1.0f;
conf.alarmB = 1.0f;
conf.encoding = 0;
conf.lostConfirmations = 1;
conf.gainBoost = 0;

lw20ServoConf servo;

Layer 3:
// Set config block could be used here. Pump makes sure entire config is written out before executing
// any independently requested commands. Just a command buffer stored in a special slot.
// Retreive config block gets all parameters.

// These are just command buffers? Could generalize structure

// Commands are attached to buffer, must still exist in memory, since all memory is allocated client side (Until layer 3).

createCommandBuffer(retries, timeout, show streaming);
setBufferedCommand(buffer, cmd, response);
// Command that can write response somewhere?

// Helper that creates a buffer with a single command.
createSingleCommand(cmd, 

beginCommandBuffer();

endCommandBuffer();

// All memory for commands here is client allocated. If your pump spans functions, make sure the commands
// are heap allocated. Command buffer structure that has X commands allocated.

// If you don't have the memory to store a command buffer there is no harm in executing commands individually.
// Command buffers purely act as a convenience to manage a single pump cycle for sending multiple commands. 
// This is exactly what the config blocks use internally.

*/

// TODO: How do timeouts work with delays to pumping?

//-------------------------------------------------------------------------
// Application Entry.
//-------------------------------------------------------------------------
int main(int args, char **argv)
{
	printf("Program Start\n");

	timespec timeLastFrame;
	clock_gettime(CLOCK_REALTIME, &timeLastFrame);

	serialOpen(&_globalSerialPort);

	lwLW20 lw20 = lw20CreateLW20();
	uint8_t inputBuffer[128];
	int32_t inputBufferSize = 0;

	//-------------------------------------------------------------------------
	// Pump until inited.
	//-------------------------------------------------------------------------
	bool initing = true;
	while (initing)
	{
		// TODO: Reset update function.
		// lw20ResetUpdate()
		lwEventLoopUpdate update;
		update.lw20 = &lw20;
		update.inputBuffer = inputBuffer;
		update.inputBufferSize = inputBufferSize;
		update.packet.length = 0;

		lwEventLoopResult result = lw20PumpEventLoop(&update);

		// If we read bytes, shift the input buffer
		if (update.bytesRead > 0)
		{
			printf("Bytes Read: %d\n", update.bytesRead);
			int32_t remaining = inputBufferSize - update.bytesRead;
			for (int i = 0; i < remaining; ++i)
			{
				inputBuffer[i] = inputBuffer[update.bytesRead + i];
			}

			inputBufferSize = remaining;
		}
		
		switch (result)
		{
			case LWELR_SEND:
			{
				printf("Send packet: (%d) [", update.packet.length);
				for (int i = 0; i < update.packet.length; ++i)
				{
					if (update.packet.buffer[i] != '\n' && update.packet.buffer[i] != '\r')
						printf("%c", update.packet.buffer[i]);
				}
				printf("]\n");
				serialWrite(&_globalSerialPort, (char*)update.packet.buffer, update.packet.length);
			} break;

			case LWELR_SLEEP:
			{
				printf("Sleep for %dms\n", update.timeMS);
				usleep(update.timeMS * 1000);
			} break;

			case LWELR_IO_WAIT:
			{
				// TODO: We are actually expecting a packet in all cases, so it seems like we could
				// just have a simple packet getter API, then feed the packet data into the pump.
				// An entire packet struct with data could processed here. ie, the pump doesn't parse at all.
				printf("IO Wait\n");

				// Constantly feed remaining data until we have a packet.
				if (inputBufferSize > 0)
				{
					printf("Had remaining\n");
					break;
				}

				// TODO: Timeout specified by pump?
				int32_t timeout = lw20_getMillis() + 1000;
				bool timedOut = false;
				while (!(timedOut = !(lw20_getMillis() < timeout)))
				{	
					inputBufferSize = serialReadNoError(&_globalSerialPort, (char*)inputBuffer, sizeof(inputBuffer));
					printf("(%d) [\n", inputBufferSize);
					if (inputBufferSize == -1)
					{
						printf("IO_WAIT Read error: %d\n", inputBufferSize);						
						initing = false;
						break;
					}
					else if (inputBufferSize != 0)
					{
						for (int i = 0; i < inputBufferSize; ++i)							
							if (inputBuffer[i] != '\n' && inputBuffer[i] != '\r')
								printf("%c", inputBuffer[i]);
						printf("]\n");
						break;
					}
				}

				if (timedOut)
				{
					initing = false;
					printf("Recv Timeout\n");
				}
			} break;

			case LWELR_INITED:
			{
				initing = false;
				printf("Inited\n");
			} break;

			case LWELR_AGAIN:
			{
			} break;

			case LWELR_FEEDBACK:			
			case LWELR_ERROR:
			case LWELR_TIMEOUT:
			case LWELR_COMPLETED:
			default:
			{
				initing = false;
				printf("Unknown event pump response\n");
			} break;
		};
	}
	//-------------------------------------------------------------------------

	printf("Done event pump\n");

	printf("Product: %s, %f, %f\n", lw20.model, lw20.firmwareVersion, lw20.softwareVersion);
	//int32_t baudRate = lw20BaudRateToInt(lw20GetComsBaudRate(&lw20));
	//printf("Baud Rate:%d\n", baudRate);
	//lw20SetComsBaudRate(&lw20, LWBR_921600);
	//lw20SaveAll(&lw20);

	//sleep(1);

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
		
		// Update all parameters.
		float distance = lw20GetDistance(&lw20, LWPT_FIRST, LWRF_RAW);
		float internalTemp = lw20GetLaserTemperature(&lw20);
		float backgroundNoise = lw20GetLaserBackgroundNoise(&lw20);
		int32_t signalStrength = lw20GetLaserSignalStrength(&lw20, LWPT_FIRST);
		
		int64_t t2 = PlatformGetMicrosecond();

		printf("Distance: %f - %dus\n", distance, (int)((t2 - t1)));
		printf("Temp: %f\n", internalTemp);
		printf("Noise: %f\n", backgroundNoise);
		printf("Strength: %d\n", signalStrength);
	}
	*/

	printf("Program End\n");
	serialClose(&_globalSerialPort);

	return 0;
}