//-------------------------------------------------------------------------
// LightWare LW20 Arudino API V0.0.0
//
// First Pass.
// Support for Arduino, PI, Windows, Linux (8bit, 32bit, 64bit).
// Using minimal C++.
// Single header, no dependencies beyond c lib.
// No dynamic memory allocations.
//
// Supported LightWare Products:
// Model: LW20 - FW: 1.0 - SW: 1.0
//-------------------------------------------------------------------------
//
// Communication must be decoupled.
// We want to support any buffer based communication: (TCP, UDP, Serial, etc...)
// Callback functions for reading and writing.
// Virtual class that has functions for read write
//
// Non-blocking can also use callbacks.

// Feed/Extract system, can only be non-blocking.

// Queue Command - Get error status.
// Update - Get command results, and error status.
// How to get command results?

// Allow C++ object method based callbacks?

// Streaming into a user specified buffer?

// You can use any method within its own thread.

//-------------------------------------------------------------------------

#ifndef LIGHTWARE_INCLUDE_LW20API_H
#define LIGHTWARE_INCLUDE_LW20API_H

// Return false on error, cancel LW20 IO operation.
// Write Size number of bytes from Buffer.
bool comsWrite(uint8_t* Buffer, int32_t Size);
// Read Size number of bytes to Buffer.
bool comsRead(uint8_t* Buffer, int32_t Size);
// Get number of bytes available to read.
int32_t comsReadAvailable();

#define LW20_FORCE_MMI_MODE_CHAR  '`'
#define LW20_FORCE_HMI_MODE_CHAR  27

#define LW20_QUERY    "?"
#define LW20_SET      "#"
#define LW20_STREAM   "$"
#define LW20_SAVE     "%"
#define LW20_TERMINAL "\r"

#define LW20_CMD_QUERY(ID)  (LW20_QUERY ID LW20_TERMINAL)
#define LW20_CMD_SET(ID)    (LW20_SET ID LW20_TERMINAL)

#define LW20_ID_PRODUCT         					"p"

#define LW20_ID_LASER_MODE      					"lm"
#define LW20_ID_LASER_DISTANCE_FIRST  		"ldf"
#define LW20_ID_LASER_DISTANCE_LAST	  		"ldl"
#define LW20_ID_LASER_OFFSET							"lo"
#define LW20_ID_LASER_ALARM_A_DISTANCE		"laa"
#define LW20_ID_LASER_ALARM_B_DISTANCE		"lab"
#define LW20_ID_LASER_ALARM_HYSTERESIS		"lah"
#define LW20_ID_LASER_ENCODING_PATTERN		"le"
#define LW20_ID_LASER_LOST_CONFIRMATIONS	"lc"
#define LW20_ID_LASER_GAIN_BOOT						"lb"

#define LW20_SERVO_CONNECTED							"sc"
#define LW20_SERVO_POSITION								"sp"
#define LW20_SERVO_LAG 										"sl"
#define LW20_SERVO_STEPS									"sr"
#define LW20_SERVO_DIR										"st"
#define LW20_SERVO_SCAN										"ss"
#define LW20_SERVO_PWM_MIN								"swl"
#define LW20_SERVO_PWM_MAX								"swh"
#define LW20_SERVO_PWM_SCALE							"sws"
#define LW20_SERVO_FOV_LOW								"sfl"
#define LW20_SERVO_FOV_HIGH								"sfh"
#define LW20_SERVO_ALARM_A_LOW						"sal"
#define LW20_SERVO_ALARM_A_HIGH						"sah"
#define LW20_SERVO_ALARM_B_LOW						"sbl"
#define LW20_SERVO_ALARM_B_HIGH						"sbh"

#define LW20_ID_COMS_BAUD_RATE  					"cb"
#define LW20_ID_COMS_TAGGING							"ct"
#define LW20_ID_COMS_I2C_ADDRESS					"ci"

//-------------------------------------------------------------------------
// Define Public Types and Function Prototypes

// LW20 Commands & Responses
enum lwCommand
{
	LWC_NONE,

	LWC_SAVE_ALL,

	LWC_PRODUCT,
	
	LWC_LASER_MODE,
	LWC_LASER_DISTANCE_FIRST,
	LWC_LASER_DISTANCE_LAST,

	LWC_COMS_BAUD_RATE,
};

enum lwPulseType
{
	LWPT_FIRST,
	LWPT_LAST,
};

enum lwReturnFilter
{
	LWRF_MEDIAN		= 0,
	LWRF_RAW 			= 1,	
	LWRF_CLOSEST		= 2,
	LWRF_FURTHEST	= 3,
};

enum lwBaudRate
{
	LWBR_9600		= 0,
	LWBR_19200	= 1,
	LWBR_38400	= 2,
	LWBR_57600	= 3,
	LWBR_115200	= 4,
	LWBR_230400	= 5,
	LWBR_460800 = 6,
	LWBR_921600 = 7,
};

struct lwLW20
{
	bool			active;	
	char 			model[8];
	float 		softwareVersion;
	float 		firmwareVersion;

	uint8_t		packetBuf[32];
	int 			packetLen = 0;
};

struct lwParser
{
	uint8_t*	packetBuf;
	int 			packetLen = 0;
	int 			packetIdx = 0;
	int 			nextChar = 0;
	int 			lexemeStart = 0;
	int 			lexemeLength = 0;
};

#ifdef LW20_API_IMPLEMENTATION

bool isCharNumber(int C)
{
	return ((C >= '0' && C <= '9') || C == '-' || C == '.');
}

bool isCharIdentifier(int C)
{
	return ((C >= 'a' && C <= 'z') || (C >= 'A' && C <= 'Z') || C == '$' || C == '%' || isCharNumber(C));
}

void getNextChar(lwParser* Parser)
{
	if (Parser->packetIdx < Parser->packetLen - 1)
		Parser->nextChar = Parser->packetBuf[Parser->packetIdx++];
	else 
		Parser->nextChar = -1;
}

bool checkIdentSingle(lwParser* Parser, const char* B)
{
	return ((Parser->packetBuf + Parser->lexemeStart)[0] == B[0]);
}

bool checkIdentDouble(lwParser* Parser, const char* B)
{
	return ((Parser->packetBuf + Parser->lexemeStart)[0] == B[0]) &&
		((Parser->packetBuf + Parser->lexemeStart)[1] == B[1]);
}

bool checkIdentTriple(lwParser* Parser, const char* B)
{
	return ((Parser->packetBuf + Parser->lexemeStart)[0] == B[0]) &&
		((Parser->packetBuf + Parser->lexemeStart)[1] == B[1]) &&
		((Parser->packetBuf + Parser->lexemeStart)[2] == B[2]);
}

bool expectPacketDelimeter(lwParser* Parser)
{
	if (Parser->nextChar == ':')
	{
		getNextChar(Parser);
		return true;
	}

	return false;
}

bool expectParamDelimeter(lwParser* Parser)
{
	if (Parser->nextChar == ',')
	{
		getNextChar(Parser);
		return true;
	}

	return false;
}

bool expectCharLiterals(lwParser* Parser, const char* Ident)
{
	while (*Ident != 0)
	{
		if (Parser->nextChar == *Ident++)
			getNextChar(Parser);
		else
			return false;
	}

	return true;
}

bool expectIdentifier(lwParser* Parser)
{
	if (isCharIdentifier(Parser->nextChar))
	{
		Parser->lexemeStart = Parser->packetIdx - 1;
		Parser->lexemeLength = 1;
		getNextChar(Parser);
		while (isCharIdentifier(Parser->nextChar))
		{
			++Parser->lexemeLength;
			getNextChar(Parser);
		}

		return true;
	}

	return false;
}

bool expectIdentifier(lwParser* Parser, char* Buffer)
{
	if (isCharIdentifier(Parser->nextChar))
	{
		(Buffer++)[0] = Parser->nextChar;
		Parser->lexemeStart = Parser->packetIdx - 1;
		Parser->lexemeLength = 1;
		getNextChar(Parser);
		while (isCharIdentifier(Parser->nextChar) || isCharNumber(Parser->nextChar))
		{
			(Buffer++)[0] = Parser->nextChar;
			++Parser->lexemeLength;
			getNextChar(Parser);
		}

		Buffer[0] = 0;
		return true;
	}

	Buffer[0] = 0;
	return false;
}

bool expectNumber(lwParser* Parser, float* Number)
{
	if (isCharNumber(Parser->nextChar))
	{
		Parser->lexemeStart = Parser->packetIdx - 1;
		
		int part = 0;
		bool neg = false;

		while (Parser->nextChar != -1 && (Parser->nextChar < '0' || Parser->nextChar > '9') && Parser->nextChar != '-' && Parser->nextChar != '.')
			getNextChar(Parser);

		if (Parser->nextChar == '-')
		{
			neg = true;
			getNextChar(Parser);
		}

		while (Parser->nextChar != -1 && !(Parser->nextChar > '9' || Parser->nextChar < '0'))
		{
			part = part * 10 + (Parser->nextChar - '0');
			getNextChar(Parser);
		}

		*Number = neg ? (float)(part * -1) : (float)part;

		if (Parser->nextChar == '.')
		{
			getNextChar(Parser);

			// TODO: Can we make this a float?
			double mul = 1;
			part = 0;

			while (Parser->nextChar != -1 && !(Parser->nextChar > '9' || Parser->nextChar < '0'))
			{
				part = part * 10 + (Parser->nextChar - '0');
				mul *= 10;
				getNextChar(Parser);
			}

			if (neg)
				*Number -= (float)part / (float)mul;
			else
				*Number += (float)part / (float)mul;
		}
		
		Parser->lexemeLength = Parser->packetIdx - 1 - Parser->lexemeStart;
		
		return true;
	}

	return false;
}

bool parseResponse(lwLW20* Lw20, lwCommand ResponseType, void* ResponseData = 0)
{
	// TODO: Are we looking for a specific resp?
	// TODO: Let streams through.

	lwParser parser = {};
	parser.packetBuf = Lw20->packetBuf;
	parser.packetLen = Lw20->packetLen;
	parser.packetIdx = 0;
	getNextChar(&parser);

	// If we're looking for streaming, or a specific response, then handle that first.
	if (!expectIdentifier(&parser)) return false;
	
	if (ResponseType == LWC_PRODUCT)
	{
		char model[8];
		float softwareVersion = 0.0f;
		float firmwareVersion = 0.0f;

		if (!checkIdentSingle(&parser, LW20_ID_PRODUCT)) return false;
		if (!expectPacketDelimeter(&parser)) return false;
		if (!expectIdentifier(&parser, model)) return false;
		if (!expectParamDelimeter(&parser)) return false;
		if (!expectNumber(&parser, &softwareVersion)) return false;
		if (!expectParamDelimeter(&parser)) return false;
		if (!expectNumber(&parser, &firmwareVersion)) return false;

		if (ResponseData)
		{
			lwLW20* lw20 = (lwLW20*)ResponseData;
			memcpy(lw20->model, model, 8);
		}
		
		return true;
	}
	else if (ResponseType == LWC_COMS_BAUD_RATE)
	{
		float baudRate = 0;
	
		if (!checkIdentSingle(&parser, LW20_ID_COMS_BAUD_RATE)) return false;
		if (!expectPacketDelimeter(&parser)) return false;
		if (!expectNumber(&parser, &baudRate)) return false;

		if (ResponseData)
			*(int32_t*)ResponseData = baudRate;
		
		return true;
	}
	else if (ResponseType == LWC_LASER_DISTANCE_FIRST || ResponseType == LWC_LASER_DISTANCE_LAST)
	{
		float distance = 0;
		float filterType = 0;
		
		if (ResponseType == LWC_LASER_DISTANCE_FIRST && !checkIdentSingle(&parser, LW20_ID_LASER_DISTANCE_FIRST)) return false;
		if (ResponseType == LWC_LASER_DISTANCE_LAST && !checkIdentSingle(&parser, LW20_ID_LASER_DISTANCE_LAST)) return false;
			
		if (!expectParamDelimeter(&parser)) return false;
		if (!expectNumber(&parser, &filterType)) return false;
		if (!expectPacketDelimeter(&parser)) return false;
		if (!expectNumber(&parser, &distance)) return false;

		if (ResponseData != 0)
			*(float*)ResponseData = distance;
		
		return true;
	}
	else if (ResponseType == LWC_SAVE_ALL)
	{
		if (!checkIdentDouble(&parser, "%p")) return false;

		return true;
	}

	return false;
}

bool readResponse(lwLW20* Lw20, lwCommand ResponseType, void* ResponseData = 0)
{
	// TODO: Add ability to repeat cmd if failed.
	//Serial.println("Waiting for response");
	
	unsigned long timeout = millis() + 1000;
	while (millis() < timeout)
	{
		if (LW20_SERIAL.available() > 0)
		{
			int c = LW20_SERIAL.read();
			
			if (c == '\n')
			{
				Lw20->packetLen = 0;
			}
			else if (c == '\r')
			{
				if (Lw20->packetLen > 0)
				{          
					bool result = parseResponse(Lw20, ResponseType, ResponseData);

					if (!result)
					{
						// TODO: Parse failed.
					}

					Lw20->packetLen = 0;
					return true;
				}
			}
			else
			{
				if (Lw20->packetLen == 32)
				{
					Serial.println("Packet overflow");
					Lw20->packetLen = 0;
				}
				else
					Lw20->packetBuf[Lw20->packetLen++] = c;
			}
		}
	}

	Serial.println("Request timeout");
	return false;
}

void flushDeviceSerial()
{
	while (LW20_SERIAL.available())
		LW20_SERIAL.read();
}

bool lw20Init(lwLW20* Lw20)
{
	Lw20->active = false;

	// Set initial state.
	// Send MMI char
	LW20_SERIAL.write(LW20_FORCE_MMI_MODE_CHAR);
	delay(32);
	
	// Stop all streaming
	LW20_SERIAL.write("$\r");
	delay(32);
	
	// Stop scanning should be part of req/resp
	LW20_SERIAL.write("#SS,0\r");
	delay(32);
	
	// Prepare for clean req/resp.
	flushDeviceSerial();

	Serial.println("Get product info");

	LW20_SERIAL.write(LW20_CMD_QUERY());
	readResponse(Lw20, LWC_PRODUCT, Lw20);

	Lw20->active = true;
	return true;
}

bool lw20Ping(lwLW20* Lw20)
{
	LW20_SERIAL.write(LW20_CMD_QUERY());
	readResponse(Lw20, LWC_PRODUCT);

	return true;
}

bool lw20GetComsBaudRate(lwLW20* Lw20, int32_t* BaudRate)
{
	int32_t baudRate = 0;

	LW20_SERIAL.write(LW20_CMD_QUERY(LW20_ID_COMS_BAUD_RATE));
	readResponse(Lw20, LWC_COMS_BAUD_RATE, &baudRate);

	if (baudRate == 0) *BaudRate = 9600;
	else if (baudRate == 1) *BaudRate = 19200;
	else if (baudRate == 2) *BaudRate = 38400;
	else if (baudRate == 3) *BaudRate = 57600;
	else if (baudRate == 4) *BaudRate = 115200;
	else if (baudRate == 5) *BaudRate = 230400;
	else if (baudRate == 6) *BaudRate = 460800;
	else if (baudRate == 7) *BaudRate = 921600;

	return true;
}

bool lw20SetComsBaudRate(lwLW20* Lw20, lwBaudRate BaudRate)
{
	LW20_SERIAL.write(LW20_SET LW20_ID_COMS_BAUD_RATE ",");
	LW20_SERIAL.print(BaudRate);
	LW20_SERIAL.write(LW20_TERMINAL);
	readResponse(Lw20, LWC_COMS_BAUD_RATE);

	return true;
}

bool lw20GetDistance(lwLW20* Lw20, lwPulseType PulseType, lwReturnFilter ReturnFilter, float* Distance)
{
	if (PulseType == LWPT_FIRST)
		LW20_SERIAL.write("?ldf,");
	else if (PulseType == LWPT_LAST)
		LW20_SERIAL.write("?ldl,");

	LW20_SERIAL.print(ReturnFilter);
	LW20_SERIAL.print("\r");

	if (PulseType == LWPT_FIRST)
	readResponse(Lw20, LWC_LASER_DISTANCE_FIRST, Distance);
  else
	readResponse(Lw20, LWC_LASER_DISTANCE_LAST, Distance);

  return true;
}

bool lw20StartScan(lwLW20* Lw20)
{
	LW20_SERIAL.write(LW20_CMD_SET("ss,1"));
	readResponse(Lw20, LWC_COMS_BAUD_RATE);

	return true;
}

bool lw20SaveAll(lwLW20* Lw20)
{
	LW20_SERIAL.write("%P\r");
	readResponse(Lw20, LWC_SAVE_ALL);
	
	return true;
}

#endif

#endif