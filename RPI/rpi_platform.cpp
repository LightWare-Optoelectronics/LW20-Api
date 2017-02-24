//-------------------------------------------------------------------------------
// All calls to linux are made here.
// This is the only file that has platform specific code.
//-------------------------------------------------------------------------------

#include "platform.h"
#include "game.h"
#include "opengl.h"
#include <unistd.h>
#include <stdlib.h>
#include "bcm_host.h"
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/poll.h>
#include <sys/eventfd.h>
#include <linux/input.h>

struct RPIContext
{
	uint32 screenWidth;
	uint32 screenHeight;
	EGLDisplay display;
	EGLSurface surface;
	EGLContext context;

	char resourceDir[64];

	char glVersion[64];
	float frameRate;
	GameState *gameState;
};

static RPIContext g_Rpi;

inline int64 PlatformGetMicrosecond()
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

int main(int args, char **argv)
{
	//getcwd(g_Rpi.resourceDir, 64);
	//strcat(g_Rpi.resourceDir, "/");
	strcpy(g_Rpi.resourceDir, "/home/pi/c_proj/");
	printf("Program Start: %d %s %s\n", args, argv[0], g_Rpi.resourceDir);

	for (int i = 0; i < args; ++i)
	{
		printf("Arg %d: %s\n", i, argv[i]);
	}

	DemoMode demoMode = DM_SINGLE_POINT;
	if (args >= 2)
	{
		demoMode = (DemoMode)(atoi(argv[1]));
	}
	printf("Demo Mode: %d\n", (int)demoMode);

	RPIContext context;

	bcm_host_init();

	// Init OpenGL
	int32 success = 0;
	EGLBoolean result;
	EGLint numConfig;

	static EGL_DISPMANX_WINDOW_T nativeWindow;

	DISPMANX_ELEMENT_HANDLE_T dispmanElement;
	DISPMANX_DISPLAY_HANDLE_T dispmanDisplay;
	DISPMANX_UPDATE_HANDLE_T dispmanUpdate;
	VC_RECT_T dstRect;
	VC_RECT_T srcRect;

	static const EGLint attributeList[] = 
	{
		EGL_RED_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 8,
		EGL_DEPTH_SIZE, 8,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_NONE
	};

	static const EGLint contextAttributes[] =
	{
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};

	EGLConfig config;

	g_Rpi.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	result = eglInitialize(g_Rpi.display, NULL, NULL);
	result = eglChooseConfig(g_Rpi.display, attributeList, &config, 1, &numConfig);
	assert(EGL_FALSE != result);
	result = eglBindAPI(EGL_OPENGL_ES_API);
	assert(EGL_FALSE != result);

	g_Rpi.context = eglCreateContext(g_Rpi.display, config, EGL_NO_CONTEXT, contextAttributes);
	assert(g_Rpi.context != EGL_NO_CONTEXT);

	uint32 displayWidth = 0;
	uint32 displayHeight = 0;
	success = graphics_get_display_size(0, &displayWidth, &displayHeight);
	assert(success >= 0);
	log("Display Size: %dx%d\n", displayWidth, displayHeight);
	// TODO: Check aspect of display size, so we can tell the renderer about it. For now, assume 16:9
	
	g_Rpi.screenWidth = 1280;
	g_Rpi.screenHeight = 720;

	dstRect.x = 0;
	dstRect.y = 0;
	dstRect.width = displayWidth;
	dstRect.height = displayHeight;

	srcRect.x = 0;
	srcRect.y = 0;
	srcRect.width = g_Rpi.screenWidth << 16;
	srcRect.height = g_Rpi.screenHeight << 16;

	VC_DISPMANX_ALPHA_T alpha;
	alpha.flags = DISPMANX_FLAGS_ALPHA_FIXED_ALL_PIXELS;
	alpha.opacity = 255;
	alpha.mask = 0;

	dispmanDisplay = vc_dispmanx_display_open(0);
	dispmanUpdate = vc_dispmanx_update_start(0);
	dispmanElement = vc_dispmanx_element_add(dispmanUpdate, dispmanDisplay,
		0, &dstRect, 0,
		&srcRect, DISPMANX_PROTECTION_NONE,
		&alpha, 0, DISPMANX_NO_ROTATE);

	nativeWindow.element = dispmanElement;
	nativeWindow.width = g_Rpi.screenWidth;
	nativeWindow.height = g_Rpi.screenHeight;
	vc_dispmanx_update_submit_sync(dispmanUpdate);

	g_Rpi.surface = eglCreateWindowSurface(g_Rpi.display, config, &nativeWindow, NULL);
	assert(g_Rpi.surface != EGL_NO_SURFACE);

	result = eglMakeCurrent(g_Rpi.display, g_Rpi.surface, g_Rpi.surface, g_Rpi.context);
	assert(EGL_FALSE != result);

	context.gameState = GameInit(demoMode);

	if (demoMode == DM_RADIAL)
	{
		PlatformStartThread();
	}

	timespec timeLastFrame;
	clock_gettime(CLOCK_REALTIME, &timeLastFrame);
		
	while (1)
	{	
		timespec timeCurrentFrame;
		clock_gettime(CLOCK_REALTIME, &timeCurrentFrame);
		timespec temp = timeDiff(timeLastFrame, timeCurrentFrame);
		int64 timeElapsed = temp.tv_sec * 1000000 + temp.tv_nsec / 1000;
		clock_gettime(CLOCK_REALTIME, &timeLastFrame);
		float elapsedTime = (float)((double)timeElapsed / 1000000.0);
		//printf("Frame Time: %fsec\n", elapsedTime);

		int64 t1 = PlatformGetMicrosecond();
		GameUpdate(context.gameState, elapsedTime);
		int64 t2 = PlatformGetMicrosecond();
		
		int64 t3 = PlatformGetMicrosecond();
		glFlush();
		eglSwapBuffers(g_Rpi.display, g_Rpi.surface);
		int64 t4 = PlatformGetMicrosecond();

		//printf("Game: %d Swap: %d\n", (int32)(t2 - t1), (int32)(t4 - t3));
	}

	printf("Program end\n");

	return 0;
}

LoadedFile PlatformLoadFile(const char *FilePath)
{
	LoadedFile loadedFile;
	
	// TODO: don't do this here fool!!
	char filePath[256];
	strcpy(filePath, g_Rpi.resourceDir);
	strcat(filePath, FilePath);

	printf("Loading file: %s\n", filePath);

	FILE *f = fopen(filePath, "rb");
	fseek(f, 0, SEEK_END);
	long size = ftell(f);
	fseek(f, 0, SEEK_SET);
	loadedFile.size = size;
	loadedFile.data = PlatformAlloc(loadedFile.size);
	fread(loadedFile.data, size, 1, f);
	fclose(f);

	return loadedFile;
}

void PlatformFreeFile(LoadedFile *File)
{
	assert(File->data != 0);
	PlatformFree(File->data);
	File->data = 0;
	File->size = 0;
}

void *PlatformAllocateMemory(uint64 Size)
{
	return malloc(Size);
}

void *PlatformAlloc(uint32 Size)
{
	return malloc(Size);
}

void PlatformFree(void *Memory)
{
	free(Memory);
}

void PlatformDebugOut(const char *Format, ...)
{
	va_list args;
	va_start(args, Format);
	vprintf(Format, args);
	va_end(args);
}

void PlatformDebugMat4(const Mat4 *Mat)
{
	PlatformDebugOut("Mat:\n");
	for (int i = 0; i < 16; ++i)
	{
		if (i % 4 == 0)
			PlatformDebugOut("\n");

		PlatformDebugOut("%f     ", Mat->e[i]);
	}
	PlatformDebugOut("\n");
}

Vec2 PlatformGetMousePos()
{
	/*
	POINT cursorPos = {};
	GetCursorPos(&cursorPos);
	ScreenToClient(g_Win32.wndH, &cursorPos);
	//DebugOut("Mouse: %d, %d\n", cursorPos.x, cursorPos.y);

	return SetVec2((float)cursorPos.x, (float)cursorPos.y);
	*/

	return SetVec2(0.0f, 0.0f);
}

int keyboardFD = -1;

int GetKeyboardKey()
{
	struct input_event ev[64];
	int size = sizeof(struct input_event);
	char name[256] = "Unknown";

	if (keyboardFD == -1)
	{
		keyboardFD = open("/dev/input/event0", O_RDONLY);
		fcntl(keyboardFD, F_SETFL, fcntl(keyboardFD, F_GETFL) | O_NONBLOCK);

		if (keyboardFD != -1)
		{
			ioctl(keyboardFD, EVIOCGNAME(sizeof(name)), name);
			printf("Connected to Keyboard: %s\n", name);
		}
	}

	if (keyboardFD != -1)
	{
		int bytesRead = read(keyboardFD, ev, size * 64);

		if (bytesRead < 0)
		{
			if (errno != EAGAIN)
			{
				printf("Keyboard Lost\n");
				//printf("Read: %d %d\n", bytesRead, errno);
				keyboardFD = -1;
			}
		}
		else
		{
			int value = ev[0].value;

			if (value != ' ' && ev[1].value == 1 && ev[1].type == 1)
			{
				printf("Key Press: [%d]\n", (ev[1].code));
				return ev[1].code;
			}
		}
	}

	return -1;
}

bool PlatformGetKey(int32 Key)
{
	return (GetKeyboardKey() == Key);
}

int32 PlatformGetKey()
{
	return GetKeyboardKey();
}

bool PlatformGetEscapeKey()
{
	int key = GetKeyboardKey();
	if (key == 1)
	{
		close(keyboardFD);
		return true;
	}

	return false;
}

bool PlatformGetMouseDown()
{
	//return (GetKeyState(VK_LBUTTON) < 0);
	return false;
}

int32 PlatformGetRand()
{
	return rand();
}

void PlatformHardQuit()
{
	printf("Application forced exit.");
	exit(1);
}

struct PlatformComPort
{
	int fd;
	bool connected;
};

bool PlatformComClose(PlatformComPort *ComPort)
{
	if (ComPort != 0 && ComPort->fd >= 0)
	{
		close(ComPort->fd);
	}

	ComPort->fd = -1;
	ComPort->connected = false;

	return true;
}

PlatformComPort *PlatformCreateComPort()
{
	PlatformComPort *port = (PlatformComPort *)PlatformAlloc(sizeof(PlatformComPort));	
	*port = {};

	return port;
}

void PlatformFreeComPort(PlatformComPort *ComPort)
{
	PlatformComClose(ComPort);
	PlatformFree(ComPort);
}

bool PlatformComOpen(PlatformComPort *ComPort)
{
	//const char *portName = "/dev/ttyUSB1";
	PlatformComClose(ComPort);

	int fd = -1;
	for (int i = 0; i < 10; ++i)
	{
		char portName[256];
		sprintf(portName, "/dev/ttyUSB%d", i);

		//printf("Attempt com connection: %s\n", portName);
		
		fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
		if (fd >= 0)
		{
			break;
		}
	}

	if (fd < 0)
	{
		//printf("Couldn't open serial port!\n");
		return false;
	}

	int speed = B115200;
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
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0; // 500ms for read wait time? Should be more like 1ms jeezuz. Better yet, alternate thread, sigh.
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

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

int32 PlatformComRead(PlatformComPort *ComPort, char *Buffer, int32 BufferSize)
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
		PlatformComClose(ComPort);
		return 0;
	}

	return readBytes;
}

/*
bool PlatformComSerialPortOpen(PlatformComPort *ComPort)
{
	PlatformComClose(ComPort);

	const char *portName = "/dev/ttyAMA0";
	int speed = B921600;
	int parity = 0;

	int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
	
	if (fd < 0)
	{
		printf("Couldn't open serial port!\n");
		return false;
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0)
	{
		printf("Error from tcgetattr\n");
		return false;
	}

	errno = 0;
	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

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

	tcflush(fd, TCIFLUSH);
	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("Error from tcsetattr\n");
		return false;
	}

	printf("Connected to %s\n", portName);
	ComPort->fd = fd;
	ComPort->connected = true;

	return true;
}
*/

int32 PlatformComReadNoError(PlatformComPort *ComPort, char *Buffer, int32 BufferSize)
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

void PlatformComWrite(PlatformComPort *ComPort, char *Buffer, int32 BufferSize)
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

bool PlatformComIsConnected(PlatformComPort *ComPort)
{
	return ComPort->connected;
}

int OpenHardwareSerial()
{
	const char *portName = "/dev/ttyAMA0";
	int speed = B921600;
	int parity = 0;

	int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);

	if (fd < 0)
	{
		printf("Couldn't open serial port!\n");
		return -1;
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0)
	{
		printf("Error from tcgetattr\n");
		return -1;
	}

	errno = 0;
	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

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

	tcflush(fd, TCIFLUSH);
	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("Error from tcsetattr\n");
		return -1;
	}

	printf("Connected to %s\n", portName);

	return fd;
}

pthread_t			gIOThread;
pthread_mutex_t		gPointMutex;
volatile int		gPointBuffer[4096];
volatile int		gPointCount = 0;
volatile int		gPointIndex = 0;

void *RunIOThread(void *Args)
{
	printf("IO Thread Running\n");

	int serialFD = OpenHardwareSerial();
	if (serialFD < 0)
	{
		printf("Invalid Serial Port\n");
		pthread_exit(0);
	}

	int pinFD = open("/sys/class/gpio/gpio18/value", O_RDONLY|O_NONBLOCK);
	if (pinFD < 0)
	{
		printf("Invalid Pin File\n");
		pthread_exit(0);
	}

	struct pollfd ufds[32];
	int fdCount = 2;

	ufds[0].fd = pinFD;
	ufds[0].events = POLLPRI;
	ufds[1].fd = serialFD;
	ufds[1].events = POLLIN;

	while (1)
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
			if (ufds[0].revents & POLLPRI)
			{
				int data;
				//lseek(pinFD, 0, SEEK_SET);
				int n = pread(pinFD, &data, 1, 0);
				//printf("pin %d %d\n", data, n);

				pthread_mutex_lock(&gPointMutex);
				gPointCount = gPointIndex;
				gPointIndex = 0;
				pthread_mutex_unlock(&gPointMutex);
			}

			if (ufds[1].revents & POLLIN)
			{
				char buffer[4096];
				int readBytes = read(serialFD, buffer, 4096);				

				pthread_mutex_lock(&gPointMutex);
				
				for (int i = 0; i < readBytes; ++i)
				{
					gPointBuffer[gPointIndex++] = buffer[i];
				
					if (gPointIndex >= 4096)
					{
						gPointIndex = 0;
						printf("Overflow\n");
					}
				}

				if (gPointCount < gPointIndex)
					gPointCount = gPointIndex;
				
				pthread_mutex_unlock(&gPointMutex);

				//printf("serial %d \n", readBytes);
			}
		}
	}

	pthread_exit(0);
}

void PlatformStartThread()
{
	pthread_mutex_init(&gPointMutex, 0);
	pthread_create(&gIOThread, 0, RunIOThread, 0);
}

int PlatformUpdateRadialBuffer(int *Buffer, int Size)
{
	int pointCount = 0;
	pthread_mutex_lock(&gPointMutex);
	pointCount = gPointCount;

	if (Size >= pointCount)
	{
		memcpy((void *)Buffer, (void *)gPointBuffer, pointCount * sizeof(int));
	}
	else
	{
		pointCount = -1;
	}
	
	pthread_mutex_unlock(&gPointMutex);

	return pointCount;
}

pid_t camPID = -1;

void PlatformExecute(char *Command)
{
	camPID = fork();
	if (camPID == 0)
	{
		system(Command);
		exit(0);
	}
	else if (camPID == -1)
	{
		printf("Failed to create child process\n");
	}
}

void PlatformExit()
{
	if (camPID != -1)
	{
		system("pkill raspivid");
	}

	exit(0);
}