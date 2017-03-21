//-------------------------------------------------------------------------
// LightWare LW20 API V0.5.0
// Written by: Robert Gowans, rob@lightware.co.za
//-------------------------------------------------------------------------

// Supported LightWare Products:
//-------------------------------------------------------------------------
// Model: LW20 - FW: 2.0 - SW: 2.0 // Added stream character for streams

// Features & Characteristics:
//-------------------------------------------------------------------------
// - Support for Arduino, RaspberryPI, Windows & Linux. (8bit, 32bit, 64bit)
// - Single header file library.
// - Zero external dependencies.
// - Zero dynamic memory allocations.
// - Minimal C++ style.

// How to use single file header only libraries:
//-------------------------------------------------------------------------
// You can include this file as normal when you would include any other 
// header file. However, you need to define the implementation in one
// C or C++ compilation unit file. You can do this with:
// 
// #define LW20_API_IMPLEMENTATION
// #include "lw20api.h"


#ifndef LIGHTWARE_INCLUDE_LW20API_H
#define LIGHTWARE_INCLUDE_LW20API_H

#define LW20_STREAM_CHANNEL_COUNT	5

#define LW20_QUERY		"?"
#define LW20_SET		"#"
#define LW20_STREAM		"$"
#define LW20_SAVE		"%"
#define LW20_TERMINAL	"\r"

#define LW20_CMD_QUERY(ID)	(LW20_QUERY ID LW20_TERMINAL)
#define LW20_CMD_SET(ID)	(LW20_SET ID LW20_TERMINAL)

#define LW20_ID_PRODUCT						"p"

#define LW20_ID_LASER_MODE					"lm"
#define LW20_ID_LASER_FIRING				"lf"
#define LW20_ID_LASER_TEMPERATURE			"lt"
#define LW20_ID_LASER_BACKGROUND_NOISE		"ln"
#define LW20_ID_LASER_DISTANCE_FIRST		"ldf"
#define LW20_ID_LASER_DISTANCE_LAST			"ldl"
#define LW20_ID_LASER_SIGNAL_STRENGTH_FIRST	"lhf"
#define LW20_ID_LASER_SIGNAL_STRENGTH_LAST	"lhl"
#define LW20_ID_LASER_OFFSET				"lo"
#define LW20_ID_LASER_ALARM_A_DISTANCE		"laa"
#define LW20_ID_LASER_ALARM_B_DISTANCE		"lab"
#define LW20_ID_LASER_ALARM_HYSTERESIS		"lah"
#define LW20_ID_LASER_ENCODING_PATTERN		"le"
#define LW20_ID_LASER_LOST_CONFIRMATIONS	"lc"
#define LW20_ID_LASER_GAIN_BOOT				"lb"

#define LW20_ID_SERVO_CONNECTED				"sc"
#define LW20_ID_SERVO_POSITION				"sp"
#define LW20_ID_SERVO_LAG					"sl"
#define LW20_ID_SERVO_STEPS					"sr"
#define LW20_ID_SERVO_DIR					"st"
#define LW20_ID_SERVO_SCAN					"ss"
#define LW20_ID_SERVO_PWM_MIN				"swl"
#define LW20_ID_SERVO_PWM_MAX				"swh"
#define LW20_ID_SERVO_PWM_SCALE				"sws"
#define LW20_ID_SERVO_FOV_LOW				"sfl"
#define LW20_ID_SERVO_FOV_HIGH				"sfh"
#define LW20_ID_SERVO_ALARM_A_LOW			"sal"
#define LW20_ID_SERVO_ALARM_A_HIGH			"sah"
#define LW20_ID_SERVO_ALARM_B_LOW			"sbl"
#define LW20_ID_SERVO_ALARM_B_HIGH			"sbh"

#define LW20_ID_ALARM_BOTH					"a"
#define LW20_ID_ALARM_A						"aa"
#define LW20_ID_ALARM_B						"ab"

#define LW20_ID_COMS_BAUD_RATE  			"cb"
#define LW20_ID_COMS_I2C_ADDRESS			"ci"
#define LW20_ID_COMS_TAGGING				"ct"

#define LW20_ID_ENERGY_POWER_CONSUMPTION	"e"

//-------------------------------------------------------------------------
// Define Public Types and Function Prototypes

// LW20 Commands & Responses
enum lwCommand
{
	LWC_NONE,
	LWC_NO_RESPONSE,

	LWC_STREAM_CLEAR,
	LWC_STREAM_1,
	LWC_STREAM_2,
	LWC_STREAM_3,
	LWC_STREAM_4,
	LWC_STREAM_5,

	LWC_SAVE_ALL,

	LWC_PRODUCT,
	
	LWC_LASER_MODE,
	LWC_LASER_FIRING,
	LWC_LASER_TEMPERATURE,
	LWC_LASER_BACKGROUND_NOISE,
	LWC_LASER_DISTANCE_FIRST,
	LWC_LASER_DISTANCE_LAST,
	LWC_LASER_SIGNAL_STRENGTH_FIRST,
	LWC_LASER_SIGNAL_STRENGTH_LAST,
	LWC_LASER_OFFSET,
	LWC_LASER_ALARM_A_DISTANCE,
	LWC_LASER_ALARM_B_DISTANCE,
	LWC_LASER_ALARM_HYSTERESIS,
	LWC_LASER_ENCODING_PATTERN,
	LWC_LASER_LOST_CONFIRMATIONS,
	LWC_LASER_GAIN_BOOST,

	LWC_SERVO_CONNECTED,
	LWC_SERVO_SCANNING,
	LWC_SERVO_POSITION,
	LWC_SERVO_LAG,
	LWC_SERVO_STEPS,
	LWC_SERVO_DIR,
	LWC_SERVO_SCAN,
	LWC_SERVO_PWM_MIN,
	LWC_SERVO_PWM_MAX,
	LWC_SERVO_PWM_SCALE,
	LWC_SERVO_FOV_LOW,
	LWC_SERVO_FOV_HIGH,
	LWC_SERVO_ALARM_A_LOW,
	LWC_SERVO_ALARM_A_HIGH,
	LWC_SERVO_ALARM_B_LOW,
	LWC_SERVO_ALARM_B_HIGH,

	LWC_ALARM_STATE_BOTH,
	LWC_ALARM_STATE_A,
	LWC_ALARM_STATE_B,

	LWC_COMS_BAUD_RATE,
	LWC_COMS_I2C_ADDRESS,
	LWC_COMS_TAGGING,

	LWC_ENERGY_POWER_CONSUMPTION,
};

enum lwPulseType
{
	LWPT_FIRST,
	LWPT_LAST,
};

enum lwReturnFilter
{
	LWRF_MEDIAN		= 0,
	LWRF_RAW 		= 1,	
	LWRF_CLOSEST	= 2,
	LWRF_FURTHEST	= 3,
};

enum lwBaudRate
{
	LWBR_9600		= 0,
	LWBR_19200		= 1,
	LWBR_38400		= 2,
	LWBR_57600		= 3,
	LWBR_115200		= 4,
	LWBR_230400		= 5,
	LWBR_460800		= 6,
	LWBR_921600		= 7,
};

enum lwModeSpeed
{
	LWMS_388		= 1,
	LWMS_194		= 2,
	LWMS_129		= 3,
	LWMS_97			= 4,
	LWMS_77			= 5,
	LWMS_64			= 6,
	LWMS_55			= 7,
	LWMS_48			= 8,
};

enum lwEventLoopStatus
{
	LWELR_SLEEP,
	LWELR_SEND_PACKET,
	LWELR_GET_PACKET,
	LWELR_FEEDBACK,
	LWELR_COMPLETED,
	LWELR_ERROR,
	LWELR_TIMEOUT,
};

enum lwSensorState
{
	LWIS_SET_MMI,
	LWIS_WAIT_MMI,
	LWIS_STOP_STREAMING,
	LWIS_WAIT_STOP_STREAMING,
	LWIS_STOP_SCANNING,
	LWIS_WAIT_STOP_SCANNING,
	LWIS_GET_PRODUCT,
	LWIS_SENT_GET_PRODUCT,
	LWIS_WAIT_GET_PRODUCT,
	LWIS_INITED,

	LWIS_SENDING_COMMAND,
	LWIS_WAITING_FOR_RESPONSE,
};

struct lwEventLoopResult
{
	lwEventLoopStatus	status;
	int32_t				timeMS;
};

enum lwResolvePacketStatus
{
	LWRPS_AGAIN,
	LWRPS_ERROR,
	LWRPS_COMPLETE,
};

struct lwProductInfo
{
	char				model[8];
	float				firmwareVersion;
	float 				softwareVersion;
};

struct lwScanSample
{
	float				angle;
	float				firstPulse;
	float				lastPulse;
};

struct lwCmdPacket
{
	lwCommand			type;
	uint8_t				buffer[32];
	int32_t				length;
};

struct lwResponsePacket
{
	lwCmdPacket			data;
	lwCommand 			type;
	bool				streaming;
	
	union 
	{
		int32_t			intValue;
		float			floatValue;
		lwProductInfo 	product;
		lwScanSample	scanSample;
	};
};

struct lwLW20
{
	lwSensorState		state;
	lwProductInfo 		product;
	lwCmdPacket 		command;
	lwResponsePacket 	response;
	lwCommand			streamCommands[LW20_STREAM_CHANNEL_COUNT];
	void*				userData;
};

struct lwParser
{
	uint8_t*			packetBuf;
	int 				packetLen;
	int 				packetIdx;
	int 				nextChar;
	int 				lexemeStart;
	int 				lexemeLength;
};

/*
struct lwEventLoopUpdate
{
	lwLW20*				lw20;
	lwCmdPacket			sendPacket;
	lwResponsePacket*	responsePacket;
	int32_t				timeMS;
};
*/

struct lwResolvePacketResult
{
	lwResolvePacketStatus status;
	int32_t bytesRead;
};

#endif

#ifdef LW20_API_IMPLEMENTATION

//-------------------------------------------------------------------------
// Helper functions.
//-------------------------------------------------------------------------
int32_t lw20BaudRateToInt(lwBaudRate BaudRate)
{
	int32_t baudTable[] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
	return baudTable[BaudRate];
}

//-------------------------------------------------------------------------
// Parsing.
//-------------------------------------------------------------------------
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

bool expectCharLiterals(lwParser* Parser, const char* Chars)
{
	while (*Chars != 0)
	{
		if (Parser->nextChar == *Chars++)
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

int32_t expectIdentifier(lwParser* Parser, char* Buffer, int32_t BufferSize)
{
	int32_t identSize = 0;	
	if ((identSize < BufferSize) && isCharIdentifier(Parser->nextChar))
	{
		++identSize;
		(Buffer++)[0] = Parser->nextChar;
		Parser->lexemeStart = Parser->packetIdx - 1;
		Parser->lexemeLength = 1;
		getNextChar(Parser);
		while ((identSize < BufferSize) && (isCharIdentifier(Parser->nextChar) || isCharNumber(Parser->nextChar)))
		{
			++identSize;
			(Buffer++)[0] = Parser->nextChar;
			++Parser->lexemeLength;
			getNextChar(Parser);
		}

		Buffer[0] = 0;
		return identSize;
	}

	Buffer[0] = 0;
	return 0;
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

bool expectNumber(lwParser* Parser, int32_t* Number)
{
	float temp;
	bool result = expectNumber(Parser, &temp);
	*Number = (int32_t)temp;
	return result;
}

bool parseResponseInt(lwParser* Parser, const char* ResponseString, int32_t* ResponseData)
{
	float value = 0;

	if (!checkIdentSingle(Parser, ResponseString)) return false;
	if (!expectPacketDelimeter(Parser)) return false;
	if (!expectNumber(Parser, &value)) return false;

	if (ResponseData)
		*ResponseData = (int32_t)value;

	return true;
}

bool parseResponseFloat(lwParser* Parser, const char* ResponseString, float* ResponseData)
{
	float value = 0;

	if (!checkIdentSingle(Parser, ResponseString)) return false;
	if (!expectPacketDelimeter(Parser)) return false;
	if (!expectNumber(Parser, &value)) return false;

	if (ResponseData)
		*ResponseData = value;

	return true;
}

bool parseResponse(lwResponsePacket* Packet)
{
	// NOTE: Parsing is a finite automata using direct evalutation of the incoming data.
	// This is useful for performance constrained platforms when parsing large loads of data.

	lwParser parser = {};
	parser.packetBuf = Packet->data.buffer;
	parser.packetLen = Packet->data.length;
	parser.packetIdx = 0;

	getNextChar(&parser);	

	char identifierBuffer[8];
	char* identBuf = identifierBuffer;

	int32_t identSize = expectIdentifier(&parser, identifierBuffer, sizeof(identifierBuffer));

	if (identSize == 0)
		return false;

	// Identify streaming packets vs stream channel responses.
	if (identBuf[0] == '$' && !(identBuf[1] >= '0' && identBuf[1] <= '9'))
	{
		--identSize;
		identBuf = identifierBuffer + 1;
		Packet->streaming = true;
	}
	else
	{
		Packet->streaming = false;
	}

	if (identSize == 1)
	{
		if (identBuf[0] == '$')
		{	
			Packet->type = LWC_STREAM_CLEAR;
			return true;
		}
		else if (identBuf[0] == 'p')
		{
			Packet->type = LWC_PRODUCT;

			if (!expectPacketDelimeter(&parser)) return false;
			if (!expectIdentifier(&parser, Packet->product.model, 8)) return false;
			if (!expectParamDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &Packet->product.softwareVersion)) return false;
			if (!expectParamDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &Packet->product.firmwareVersion)) return false;

			return true;
		}
		else if (identBuf[0] == 'a')
		{
			Packet->type = LWC_ALARM_STATE_BOTH;
			
			int32_t alarmA;
			int32_t alarmB;

			if (!expectPacketDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &alarmA)) return false;
			if (!expectParamDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &alarmB)) return false;

			Packet->intValue = (alarmA << 1) | alarmB;

			return true;	
		}
	}
	else if (identSize == 2)
	{
		if (identBuf[0] == '$')
		{
			if (identBuf[1] == '1') Packet->type = LWC_STREAM_1;
			else if (identBuf[1] == '2') Packet->type = LWC_STREAM_2;
			else if (identBuf[1] == '3') Packet->type = LWC_STREAM_3;
			else if (identBuf[1] == '4') Packet->type = LWC_STREAM_4;
			else if (identBuf[1] == '5') Packet->type = LWC_STREAM_5;
			else return false;
			
			return true;
		}
		else if (identBuf[0] == 'l')
		{
			if (identBuf[1] == 'm')
			{
				Packet->type = LWC_LASER_MODE;
				
				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;	
			}
			if (identBuf[1] == 't')
			{
				Packet->type = LWC_LASER_TEMPERATURE;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
		}
		else if (identBuf[0] == 's')
		{
			if (identBuf[1] == 's')
			{
				if (Packet->streaming)
				{
					Packet->type = LWC_SERVO_SCAN;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->scanSample.angle)) return false;
					if (!expectParamDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->scanSample.firstPulse)) return false;
					if (!expectParamDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->scanSample.lastPulse)) return false;

					return true;
				}
				else
				{
					Packet->type = LWC_SERVO_SCANNING;
					return true;
				}
			}
			else if (identBuf[1] == 'c')
			{
				Packet->type = LWC_SERVO_CONNECTED;
				
				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'r')
			{
				Packet->type = LWC_SERVO_STEPS;
				
				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'l')
			{
				Packet->type = LWC_SERVO_LAG;
				
				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'p')
			{
				Packet->type = LWC_SERVO_POSITION;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
		}
	}
	else if (identSize == 3)
	{
		if (identBuf[0] == 'l')
		{
			if (identBuf[1] == 'a')
			{
				if (identBuf[2] == 'a' || identBuf[2] == 'b')
				{
					if (identBuf[2] == 'a') Packet->type = LWC_LASER_ALARM_A_DISTANCE;
					else if (identBuf[2] == 'b') Packet->type = LWC_LASER_ALARM_B_DISTANCE;
					
					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->floatValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'd')
			{
				if (identBuf[2] == 'f' || identBuf[2] == 'l')
				{
					if (identBuf[2] == 'f') Packet->type = LWC_LASER_DISTANCE_FIRST;
					else if (identBuf[2] == 'l') Packet->type = LWC_LASER_DISTANCE_LAST;

					int filterType = 0;

					if (!expectParamDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &filterType)) return false;
					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->floatValue)) return false;

					return true;
				}
			}
		}
		else if (identBuf[0] == 's')
		{
			if (identBuf[1] == 'w')
			{
				if (identBuf[2] == 'l' || identBuf[2] == 'h' || identBuf[2] == 's')
				{
					if (identBuf[2] == 'l') Packet->type = LWC_SERVO_PWM_MIN;
					else if (identBuf[2] == 'h') Packet->type = LWC_SERVO_PWM_MAX;
					else if (identBuf[2] == 's') Packet->type = LWC_SERVO_PWM_SCALE;
					
					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->floatValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'f')
			{
				if (identBuf[2] == 'l' || identBuf[2] == 'h')
				{
					if (identBuf[2] == 'l') Packet->type = LWC_SERVO_FOV_LOW;
					else if (identBuf[2] == 'h') Packet->type = LWC_SERVO_FOV_HIGH;
				
					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->intValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'a')
			{
				if (identBuf[2] == 'l' || identBuf[2] == 'h')
				{
					if (identBuf[2] == 'l') Packet->type = LWC_SERVO_ALARM_A_LOW;
					else if (identBuf[2] == 'h') Packet->type = LWC_SERVO_ALARM_A_HIGH;
				
					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->intValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'b')
			{
				if (identBuf[2] == 'l' || identBuf[2] == 'h')
				{
					if (identBuf[2] == 'l') Packet->type = LWC_SERVO_ALARM_B_LOW;
					else if (identBuf[2] == 'h') Packet->type = LWC_SERVO_ALARM_B_HIGH;
				
					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->intValue)) return false;

					return true;
				}
			}
		}
	}

	return false;
}

//-------------------------------------------------------------------------
// Packet Building.
//-------------------------------------------------------------------------
// NOTE: We always assume there is enough space in the buffer to write.

void packetWriteChar(lwCmdPacket* Packet, char Char)
{
	Packet->buffer[Packet->length++] = Char;
}

void packetWriteString(lwCmdPacket* Packet, const char* String)
{
	while (*String)
		Packet->buffer[Packet->length++] = String++[0];
}

void packetWriteDigits(lwCmdPacket* Packet, int32_t Number)
{
	int32_t startIndex = Packet->length;

	if (Number == 0)
	{
		Packet->buffer[Packet->length++] = '0';
	}
	else
	{
		while (Number > 0)
		{
			Packet->buffer[Packet->length++] = (Number % 10) + '0';
			Number /= 10; 
		}

		int32_t length = Packet->length - startIndex;
		int32_t halfLength = length / 2;

		for (int32_t i = 0; i < halfLength; ++i)
		{
			uint8_t temp = Packet->buffer[startIndex + i];
			Packet->buffer[startIndex + i] = Packet->buffer[startIndex + (length - i) - 1];
			Packet->buffer[startIndex + (length - i) - 1] = temp;
		}
	}
}

void packetWriteFloat(lwCmdPacket* Packet, float Number)
{
	if (Number < 0)
		packetWriteChar(Packet, '-');

	int32_t whole = (int32_t)Number;
	int32_t frac = (int32_t)((Number - (float)whole) * 100.0f);

	packetWriteDigits(Packet, whole);
	packetWriteChar(Packet, '.');
	packetWriteDigits(Packet, frac);
}

void packetWriteInt(lwCmdPacket* Packet, int32_t Number)
{
	if (Number < 0)
		packetWriteChar(Packet, '-');
	
	packetWriteDigits(Packet, Number * (Number < 0 ? -1 : 1));
}

void packetClear(lwCmdPacket* Packet)
{
	Packet->length = 0;
	Packet->type = LWC_NONE;
}

void packetWriteProduct(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?\r");
	Packet->type = LWC_PRODUCT;
}

void packetWriteLaserMode(lwCmdPacket* Packet, int ModeSpeed)
{
	packetClear(Packet);
	packetWriteString(Packet, "#lm,");
	packetWriteInt(Packet, ModeSpeed);
	packetWriteString(Packet, LW20_TERMINAL);
	Packet->type = LWC_LASER_MODE;
}

void packetWriteDistanceFirst(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "#ldf\r");
	Packet->type = LWC_LASER_DISTANCE_FIRST;
}

void packetWriteDistanceLast(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "#ldl\r");
	Packet->type = LWC_LASER_DISTANCE_LAST;
}

//-------------------------------------------------------------------------
// Layer 1.
//-------------------------------------------------------------------------

lwLW20 lw20CreateLW20()
{
	lwLW20 lw20 = {};
	return lw20;
}

lwResolvePacketResult lw20ResolvePacket(lwResponsePacket* Packet, uint8_t* Buffer, int32_t BufferSize)
{
	// TODO: We don't report errors here yet.
	lwResolvePacketResult result = {};

	for (int i = 0; i < BufferSize; ++i)
	{
		result.bytesRead++;

		uint8_t c = Buffer[i];
		if (c == '\n')
		{
			Packet->data.length = 0;
		}
		else if (c == '\r')
		{
			if (Packet->data.length > 0)
			{
				if (parseResponse(Packet))
				{
					result.status = LWRPS_COMPLETE;
				}
				else
				{
					Packet->data.length = 0;
					result.status = LWRPS_AGAIN;
				}

				break;
			}
		}
		else
		{
			if (Packet->data.length >= 32)
			{
				Packet->data.length = 0;
				result.status = LWRPS_AGAIN;
				break;
			}
			else
			{
				Packet->data.buffer[Packet->data.length++] = c;
			}
		}
	}

	return result;
}

lwEventLoopResult lw20PumpEventLoop(lwLW20* Lw20)
{
	lwEventLoopResult result = {};
	result.status = LWELR_COMPLETED;
	lwCmdPacket* packet = &Lw20->command;
	
	if (Lw20->state >= LWIS_INITED)
	{
		if (Lw20->state == LWIS_INITED)
		{
			if (Lw20->command.type != LWC_NONE)
			{
				Lw20->state = LWIS_SENDING_COMMAND;
				result.status = LWELR_SEND_PACKET;
			}
			else
			{
				if (Lw20->response.streaming)
				{
					Lw20->response.streaming = false;
					result.status = LWELR_FEEDBACK;
				}
				else
				{
					result.status = LWELR_COMPLETED;
				}
			}
		}
		else if (Lw20->state == LWIS_SENDING_COMMAND)
		{
			Lw20->state = LWIS_WAITING_FOR_RESPONSE;
			result.status = LWELR_GET_PACKET;
		}
		else if (Lw20->state == LWIS_WAITING_FOR_RESPONSE)
		{
			if (Lw20->response.type == Lw20->command.type)
			{
				Lw20->state = LWIS_INITED;
				Lw20->command.type = LWC_NONE;
				result.status = LWELR_COMPLETED;
			}
			else
			{
				result.status = LWELR_GET_PACKET;
			}
		}
	}
	else
	{
		if (Lw20->state == LWIS_SET_MMI)
		{
			Lw20->state = LWIS_WAIT_MMI;
			packetClear(packet);
			packetWriteChar(packet, '`');
			result.status = LWELR_SEND_PACKET;
		}
		else if (Lw20->state == LWIS_WAIT_MMI)
		{
			Lw20->state = LWIS_STOP_STREAMING;
			result.timeMS = 100;
			result.status = LWELR_SLEEP;
		}
		else if (Lw20->state == LWIS_STOP_STREAMING)
		{
			Lw20->state = LWIS_WAIT_STOP_STREAMING;
			packetClear(packet);
			packetWriteString(packet, "$\r");
			result.status = LWELR_SEND_PACKET;
		}
		else if (Lw20->state == LWIS_WAIT_STOP_STREAMING)
		{
			Lw20->state = LWIS_STOP_SCANNING;
			result.timeMS = 100;
			result.status = LWELR_SLEEP;
		}
		else if (Lw20->state == LWIS_STOP_SCANNING)
		{
			Lw20->state = LWIS_WAIT_STOP_SCANNING;
			packetClear(packet);
			packetWriteString(packet, "#SS,0\r");
			result.status = LWELR_SEND_PACKET;
		}
		else if (Lw20->state == LWIS_WAIT_STOP_SCANNING)
		{
			Lw20->state = LWIS_GET_PRODUCT;
			result.timeMS = 100;
			result.status = LWELR_SLEEP;
		}
		else if (Lw20->state == LWIS_GET_PRODUCT)
		{
			Lw20->state = LWIS_SENT_GET_PRODUCT;
			packetClear(packet);
			packetWriteString(packet, "?\r");
			result.status = LWELR_SEND_PACKET;
		}
		else if (Lw20->state == LWIS_SENT_GET_PRODUCT)		
		{
			Lw20->state = LWIS_WAIT_GET_PRODUCT;
			result.status = LWELR_GET_PACKET;
		}
		else if (Lw20->state == LWIS_WAIT_GET_PRODUCT)
		{
			if (Lw20->response.type == LWC_PRODUCT)
			{
				Lw20->product = Lw20->response.product;
				Lw20->state = LWIS_INITED;
				Lw20->command.type = LWC_NONE;
				Lw20->response.type = LWC_NONE;
				result.status = LWELR_COMPLETED;
			}
			else
			{
				result.status = LWELR_GET_PACKET;
			}
		}
	}	

	return result;
}

//-------------------------------------------------------------------------
// Layer 2 Blocking.
//-------------------------------------------------------------------------

// Sensor Context for data stores and communication callbacks.
// Communication context? Data is input buffer, callbacks for read/write.

typedef bool(*lw20SendPacketCallback)(lwLW20* Lw20, lwCmdPacket* Packet);
typedef bool(*lw20GetPacketCallback)(lwLW20* Lw20, lwResponsePacket* Packet);
typedef bool(*lw20SleepCallback)(lwLW20* Lw20, int32_t TimeMS);
typedef bool(*lw20StreamCallback)(lwLW20* Lw20, lwResponsePacket* Packet);

struct lwServiceContext
{
	lw20SendPacketCallback	sendPacketCallback;
	lw20GetPacketCallback	getPacketCallback;
	lw20SleepCallback		sleepCallback;
	lw20StreamCallback		streamCallback;
};

// Event Loop Execution
bool runEventLoop(lwLW20* Lw20, lwServiceContext* Service, bool Streaming = false)
{
	while (true)
	{
		lwEventLoopResult result = lw20PumpEventLoop(Lw20);
		
		switch (result.status)
		{
			case LWELR_SLEEP:
			{
				Service->sleepCallback(Lw20, result.timeMS);
			} break;

			case LWELR_SEND_PACKET:
			{
				if (!Service->sendPacketCallback(Lw20, &Lw20->command))
					return false;
			} break;

			case LWELR_GET_PACKET:
			{
				if (!Service->getPacketCallback(Lw20, &Lw20->response))
					return false;
			} break;

			case LWELR_ERROR:
			case LWELR_TIMEOUT:
			{
				return false;
			} break;

			case LWELR_FEEDBACK:
			{
				if (Streaming)
				{
					Service->streamCallback(Lw20, &Lw20->response);
					Service->getPacketCallback(Lw20, &Lw20->response);
				}
				else
				{
					//std::cout << "Completed Event Loop with Feedback\n";
					return true;
				}

			} break;

			case LWELR_COMPLETED:
			{	
				if (Streaming)
				{
					Service->getPacketCallback(Lw20, &Lw20->response);
				}
				else
				{
					//std::cout << "Completed Event Loop\n";
					return true;
				}

			} break;
		};
	}
}

void executeCommand(lwLW20* Lw20, lwServiceContext* Service, const char* Command, lwCommand ResponseType)
{
	packetClear(&Lw20->command);
	packetWriteString(&Lw20->command, Command);
	Lw20->command.type = ResponseType;
	runEventLoop(Lw20, Service);
}

void executeCommand(lwLW20* Lw20, lwServiceContext* Service, lwResponsePacket* Response)
{

}

#endif