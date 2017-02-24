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
// Serial Communication.
//-------------------------------------------------------------------------
struct lwSerialPort
{
	int fd;
	bool connected;
};

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

	int speed = B230400;
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

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_iflag &= ~IGNBRK;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1; // TODO: Check this: 500ms for read wait time? Should be more like 1ms jeezuz. Better yet, alternate thread, sigh.
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

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

	if (readBytes == 0)
		printf("No Data: %d Error %d (%s)\n", readBytes, errno, strerror(errno));

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
// Application Entry.
//-------------------------------------------------------------------------
int main(int args, char **argv)
{
	printf("Program Start\n");

	timespec timeLastFrame;
	clock_gettime(CLOCK_REALTIME, &timeLastFrame);

	lwSerialPort lw20Port;

	serialOpen(&lw20Port);

	char queryProduct[] = "?p\r";

	while (1)
	{	
		timespec timeCurrentFrame;
		clock_gettime(CLOCK_REALTIME, &timeCurrentFrame);
		timespec temp = timeDiff(timeLastFrame, timeCurrentFrame);
		int64_t timeElapsed = temp.tv_sec * 1000000 + temp.tv_nsec / 1000;
		clock_gettime(CLOCK_REALTIME, &timeLastFrame);
		float elapsedTime = (float)((double)timeElapsed / 1000000.0);		

		int64_t t1 = PlatformGetMicrosecond();
		printf("Frame Time: %fsec\n", elapsedTime);
		
		serialWrite(&lw20Port, queryProduct, sizeof(queryProduct) - 1);
		usleep(40000);
		
		char recvBuffer[4096];
		int32_t recvLen = serialReadNoError(&lw20Port, recvBuffer, sizeof(recvBuffer));
		printf("Recv: %d\n", recvLen);

		for (int i = 0; i < recvLen; ++i)
		{
			printf("%c", recvBuffer[i]);
		}

		printf("\n");

		int64_t t2 = PlatformGetMicrosecond();

		//sleep(1);
	}
	
	printf("Program End\n");
	serialClose(&lw20Port);

	return 0;
}