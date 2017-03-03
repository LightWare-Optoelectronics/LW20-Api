//-------------------------------------------------------------------------
// LightWare LW20 API V0.1.0
//
// First Pass.
// Support for Arduino, PI, Windows, Linux (8bit, 32bit, 64bit).
// Using minimal C++.
// Single header, no dependencies beyond c lib. Only memset?
// No dynamic memory allocations.
//
// Supported LightWare Products:
// Model: LW20 - FW: 1.0 - SW: 1.0
//-------------------------------------------------------------------------
//
// Communication must be decoupled.
// We want to support any byte based communication: (TCP, UDP, Serial, I2c, etc...)
// Callback functions for reading and writing. (At the moment just extrnally declared c functions.)
// Allow C++ object method based callbacks?
// Maybe virtual class that has functions for read write
//
// Event based. Layered usage approach.
//
// TODO: Need to fix the horrible way to declare and deal with new commands.
// Don't want to waste mem/perf for tables, so probably just codespace with
// macros to help quell the madness.
//
// TOOD: Append lw20 to all functions for "namespacing"?
//-------------------------------------------------------------------------

#ifndef LIGHTWARE_INCLUDE_LW20API_H
#define LIGHTWARE_INCLUDE_LW20API_H

#define LW20_FORCE_MMI_MODE_CHAR	'`'
#define LW20_FORCE_HMI_MODE_CHAR	'\0x1B'

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
	LWMS_388	= 1,
	LWMS_194	= 2,
	LWMS_129	= 3,
	LWMS_97		= 4,
	LWMS_77		= 5,
	LWMS_64		= 6,
	LWMS_55		= 7,
	LWMS_48		= 8,
};

enum lwError
{
	LWE_NONE,
	LWE_NOT_CONNECTED,
	LWE_TIMEOUT,
	LWE_NO_RESPONSE,
	LWE_INVALID_VALUE,
	LWE_OUTPUT_BUFFER_TOO_SMALL,
};

enum lwEventLoopResult
{
	LWELR_GET_PACKET,
	LWELR_SLEEP,
	LWELR_FEEDBACK,
	LWELR_SEND,
	LWELR_AGAIN,
	LWELR_ERROR,
	LWELR_TIMEOUT,
	LWELR_COMPLETED,
	LWELR_INITED,
};

enum lwInitState
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

enum lwResolvePacketStatus
{
	LWRPS_AGAIN,
	LWRPS_ERROR,
	LWRPS_COMPLETE,
};

struct lwProductInfo
{
	char 		model[8];
	float 		firmwareVersion;
	float 		softwareVersion;

};

struct lwLW20
{
	bool		active;
	lwInitState	initState;

	lwProductInfo product;

	uint8_t		packetBuf[32];
	int 		packetLen;

	// TODO: Need this?
	lwError		error;
	void*		userData;

	lwCommand	sendCommand;
};

struct lwParser
{
	uint8_t*	packetBuf;
	int 		packetLen;
	int 		packetIdx;
	int 		nextChar;
	int 		lexemeStart;
	int 		lexemeLength;
};

struct lwCmdPacket
{
	uint8_t	buffer[32];
	int32_t	length;
};

struct lwResponsePacket
{
	lwCmdPacket		data;
	lwCommand 		type;
	
	union 
	{
		int32_t			intValue;
		float			floatValue;
		lwProductInfo 	product;
	};
};

struct lwEventLoopUpdate
{
	lwLW20*				lw20;
	lwCmdPacket			sendPacket;
	lwResponsePacket*	responsePacket;
	int32_t				timeMS;
};

#endif

#ifdef LW20_API_IMPLEMENTATION

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

// NOTE: Max includes null term.
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
		while ((identSize < BufferSize) > 0 && (isCharIdentifier(Parser->nextChar) || isCharNumber(Parser->nextChar)))
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
		*ResponseData = value;

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
	lwParser parser = {};
	parser.packetBuf = Packet->data.buffer;
	parser.packetLen = Packet->data.length;
	parser.packetIdx = 0;

	getNextChar(&parser);	

	char identBuf[8];

	// NOTE: We include numbers in the identifier.
	int32_t identSize = expectIdentifier(&parser, identBuf, sizeof(identBuf));

	if (identSize == 0)
		return false;

	/*
	printf("Got packet ident: (%d) [", identSize);

	for (int i = 0; i < identSize; ++i)
	{
		printf("%c", identBuf[i]);
	}	

	printf("]\n");
	*/

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
		if (identBuf[0] == 'l')
		{
			if (identBuf[1] == 'm')
			{
				Packet->type = LWC_LASER_MODE;
				
				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;	
			}
		}
		else if (identBuf[0] == 's')
		{
			if (identBuf[1] == 's')
			{
				Packet->type = LWC_SERVO_SCANNING;
				return true;
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

	// Check which packet we are trying to parse.
	
	/*
	if (ResponseType == LWC_PRODUCT)
	{
		char model[8];
		float softwareVersion = 0.0f;
		float firmwareVersion = 0.0f;

		if (!checkIdentSingle(&parser, LW20_ID_PRODUCT)) return false;
		if (!expectPacketDelimeter(&parser)) return false;
		if (!expectIdentifier(&parser, model, sizeof(model))) return false;
		if (!expectParamDelimeter(&parser)) return false;
		if (!expectNumber(&parser, &softwareVersion)) return false;
		if (!expectParamDelimeter(&parser)) return false;
		if (!expectNumber(&parser, &firmwareVersion)) return false;

		if (ResponseData)
		{
			lwLW20* lw20 = (lwLW20*)ResponseData;
			memcpy(lw20->model, model, 8);
			lw20->firmwareVersion = firmwareVersion;
			lw20->softwareVersion = softwareVersion;
		}
		
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
	
	// Laser
	else if (ResponseType == LWC_LASER_MODE) { return parseResponseInt(&parser, LW20_ID_LASER_MODE, (int32_t*)ResponseData); }
	else if (ResponseType == LWC_LASER_FIRING) { return parseResponseInt(&parser, LW20_ID_LASER_FIRING, (int32_t*)ResponseData); }
	else if (ResponseType == LWC_LASER_TEMPERATURE) { return parseResponseFloat(&parser, LW20_ID_LASER_TEMPERATURE, (float*)ResponseData); }
	else if (ResponseType == LWC_LASER_BACKGROUND_NOISE) { return parseResponseFloat(&parser, LW20_ID_LASER_BACKGROUND_NOISE, (float*)ResponseData); }
	else if (ResponseType == LWC_LASER_SIGNAL_STRENGTH_FIRST) { return parseResponseInt(&parser, LW20_ID_LASER_SIGNAL_STRENGTH_FIRST, (int32_t*)ResponseData); }
	else if (ResponseType == LWC_LASER_SIGNAL_STRENGTH_LAST) { return parseResponseInt(&parser, LW20_ID_LASER_SIGNAL_STRENGTH_LAST, (int32_t*)ResponseData); }
	else if (ResponseType == LWC_LASER_OFFSET) { return parseResponseFloat(&parser, LW20_ID_LASER_OFFSET, (float*)ResponseData); }
	
	// Coms
	else if (ResponseType == LWC_COMS_BAUD_RATE) { return parseResponseInt(&parser, LW20_ID_COMS_BAUD_RATE, (int32_t*)ResponseData); }

	else if (ResponseType == LWC_SAVE_ALL)
	{
		if (!checkIdentDouble(&parser, "%p")) return false;

		return true;
	}
	*/

	return false;
}

//-------------------------------------------------------------------------
// Packet Building.
//-------------------------------------------------------------------------
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

// Writes a floating point number with 2 decimal places.
// We always assume there is enough space in the buffer.
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
}

void sendSimpleCmd(lwLW20* Lw20, lwCommand ResponseType, const char* Cmd, void* Result = 0)
{
	lwCmdPacket packet = {};
	packetWriteString(&packet, Cmd);
	//packetSend(&packet);
	//readResponse(Lw20, ResponseType, Result);
}

void sendSimpleCmdSetInt(lwLW20* Lw20, lwCommand ResponseType, const char* Cmd, int Value)
{
	lwCmdPacket packet = {};
	packetWriteString(&packet, Cmd);
	packetWriteInt(&packet, Value);
	packetWriteString(&packet, LW20_TERMINAL);
	//packetSend(&packet);
	//readResponse(Lw20, ResponseType);
}

int32_t sendSimpleCmdInt(lwLW20* Lw20, lwCommand ResponseType, const char* Cmd)
{
	int32_t result = 0;
	sendSimpleCmd(Lw20, ResponseType, Cmd, &result);
	return result;
}

float sendSimpleCmdFloat(lwLW20* Lw20, lwCommand ResponseType, const char* Cmd)
{
	float result = 0;
	sendSimpleCmd(Lw20, ResponseType, Cmd, &result);
	return result;
}

//-------------------------------------------------------------------------
// Global.
//-------------------------------------------------------------------------
void lw20SaveAll(lwLW20* Lw20)
{
	sendSimpleCmd(Lw20, LWC_SAVE_ALL, "%p\r");
}

//-------------------------------------------------------------------------
// Product.
//-------------------------------------------------------------------------
void lw20Product(lwLW20* Lw20)
{
	sendSimpleCmd(Lw20, LWC_PRODUCT, "?\r", Lw20);
}

//-------------------------------------------------------------------------
// Laser.
//-------------------------------------------------------------------------
int32_t lw20GetLaserMode(lwLW20* Lw20)
{
	return sendSimpleCmdInt(Lw20, LWC_LASER_MODE, LW20_CMD_QUERY(LW20_ID_LASER_MODE));
}

int32_t lw20GetLaserFiring(lwLW20* Lw20)
{
	return sendSimpleCmdInt(Lw20, LWC_LASER_FIRING, LW20_CMD_QUERY(LW20_ID_LASER_FIRING));
}

float lw20GetLaserTemperature(lwLW20* Lw20)
{
	return sendSimpleCmdFloat(Lw20, LWC_LASER_TEMPERATURE, LW20_CMD_QUERY(LW20_ID_LASER_TEMPERATURE));
}

float lw20GetLaserBackgroundNoise(lwLW20* Lw20)
{
	return sendSimpleCmdFloat(Lw20, LWC_LASER_BACKGROUND_NOISE, LW20_CMD_QUERY(LW20_ID_LASER_BACKGROUND_NOISE));
}

float lw20GetDistance(lwLW20* Lw20, lwPulseType PulseType, lwReturnFilter ReturnFilter)
{
	lwCmdPacket packet = {};

	if (PulseType == LWPT_FIRST)
		packetWriteString(&packet, "?ldf,");
	else if (PulseType == LWPT_LAST)
		packetWriteString(&packet, "?ldl,");

	packetWriteInt(&packet, (uint32_t)ReturnFilter);
	packetWriteString(&packet, LW20_TERMINAL);
	//packetSend(&packet);

	float result = 0.0f;

	/*
	if (PulseType == LWPT_FIRST)
		readResponse(Lw20, LWC_LASER_DISTANCE_FIRST, &result);
	else
		readResponse(Lw20, LWC_LASER_DISTANCE_LAST, &result);
	*/

	return result;
}

int32_t lw20GetLaserSignalStrength(lwLW20* Lw20, lwPulseType PulseType)
{
	if (PulseType == LWPT_FIRST)
		return sendSimpleCmdInt(Lw20, LWC_LASER_SIGNAL_STRENGTH_FIRST, LW20_CMD_QUERY(LW20_ID_LASER_SIGNAL_STRENGTH_FIRST));
	else if (PulseType == LWPT_LAST)
		return sendSimpleCmdInt(Lw20, LWC_LASER_SIGNAL_STRENGTH_LAST, LW20_CMD_QUERY(LW20_ID_LASER_SIGNAL_STRENGTH_LAST));

	return 0;
}

//-------------------------------------------------------------------------
// Servo.
//-------------------------------------------------------------------------

// These functions are more like helpers that sit on top of the raw protocol?
void lw20StartScan(lwLW20* Lw20)
{
	sendSimpleCmd(Lw20, LWC_SAVE_ALL, "#ss,1\r");
}

void lw20StopScan(lwLW20* Lw20)
{
	sendSimpleCmd(Lw20, LWC_SAVE_ALL, "#ss,1\r");
}

//-------------------------------------------------------------------------
// Alarms.
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Communications.
//-------------------------------------------------------------------------

lwBaudRate lw20GetComsBaudRate(lwLW20* Lw20)
{
	return (lwBaudRate)sendSimpleCmdInt(Lw20, LWC_COMS_BAUD_RATE, LW20_CMD_QUERY(LW20_ID_COMS_BAUD_RATE));
}

void lw20SetComsBaudRate(lwLW20* Lw20, lwBaudRate BaudRate)
{
	sendSimpleCmdSetInt(Lw20, LWC_COMS_BAUD_RATE, LW20_SET LW20_ID_COMS_BAUD_RATE ",", BaudRate);
}

//-------------------------------------------------------------------------
// Energy.
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Main public facing functions.
//-------------------------------------------------------------------------
int32_t lw20BaudRateToInt(lwBaudRate BaudRate)
{
	int32_t baudTable[] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
	return baudTable[BaudRate];
}

bool lw20CheckError(lwLW20* Lw20)
{
	return false;
}

bool lw20ClearError(lwLW20* Lw20)
{
	return false;
}

lwLW20 lw20CreateLW20()
{
	lwLW20 lw20 = {};
	return lw20;
}

struct lwPumpPacketResult
{
	bool gotPacket;
	int32_t bytesRead;
};

lwPumpPacketResult pumpRecvPacket(lwLW20* Lw20, uint8_t* Buffer, int32_t BufferSize)
{
	// TODO: Report streaming packets too?
	lwPumpPacketResult result = {};

	//printf("Parse\n");
	
	for (int i = 0; i < BufferSize; ++i)
	{
		result.bytesRead++;

		uint8_t c = Buffer[i];
		if (c == '\n')
		{
			Lw20->packetLen = 0;
		}
		else if (c == '\r')
		{
			if (Lw20->packetLen > 0)
			{
				// Got packet.
				//bool result = parseResponse(Lw20, ResponseType, ResponseData);
				//printf("Got Packet %d\n", Lw20->packetLen);
				// TODO: Set number of bytes read from buffer so client can advance buffer ptr.				
				result.gotPacket = true;
				return result;
			}
		}
		else
		{
			if (Lw20->packetLen >= 32)
			{
				//printf("Packet overflow\n");
				Lw20->packetLen = 0;
			}
			else
			{
				Lw20->packetBuf[Lw20->packetLen++] = c;
			}
		}
	}

	return result;
}

lwEventLoopResult lw20PumpEventLoop(lwEventLoopUpdate* Update)
{
	lwLW20* lw20 = Update->lw20;
	lwCmdPacket* packet = &Update->sendPacket;
	
	if (lw20->initState >= LWIS_INITED)
	{
		if (lw20->initState == LWIS_INITED)
		{
			if (lw20->sendCommand == LWC_NONE)
			{
				return LWELR_COMPLETED;
			}
			else
			{
				lw20->initState = LWIS_SENDING_COMMAND;
				//packetClear(packet);
				//packetWriteString(packet, (const char*)lw20->packetBuf);
				memcpy(packet, (lwCmdPacket*)&lw20->packetBuf, sizeof(lwCmdPacket));
				return LWELR_SEND;
			}
		}
		else if (lw20->initState == LWIS_SENDING_COMMAND)
		{
			lw20->initState = LWIS_WAITING_FOR_RESPONSE;
			return LWELR_GET_PACKET;
		}
		else if (lw20->initState == LWIS_WAITING_FOR_RESPONSE)
		{
			if (Update->responsePacket->type == lw20->sendCommand)
			{
				lw20->initState = LWIS_INITED;
				
				if (lw20->sendCommand != LWC_NO_RESPONSE)
				{
					lw20->sendCommand = LWC_NONE;
					return LWELR_FEEDBACK;
				}
				
				lw20->sendCommand = LWC_NONE;
				return LWELR_COMPLETED;
			}

			return LWELR_GET_PACKET;
		}

		return LWELR_COMPLETED;
	}
	else
	{
		if (lw20->initState == LWIS_SET_MMI)
		{
			lw20->initState = LWIS_WAIT_MMI;
			packetClear(packet);
			packetWriteChar(packet, LW20_FORCE_MMI_MODE_CHAR);
			return LWELR_SEND;
		}
		else if (lw20->initState == LWIS_WAIT_MMI)
		{
			lw20->initState = LWIS_STOP_STREAMING;
			Update->timeMS = 100;
			return LWELR_SLEEP;
		}
		else if (lw20->initState == LWIS_STOP_STREAMING)
		{
			lw20->initState = LWIS_WAIT_STOP_STREAMING;
			packetClear(packet);
			packetWriteString(packet, "$\r");
			return LWELR_SEND;
		}
		else if (lw20->initState == LWIS_WAIT_STOP_STREAMING)
		{
			lw20->initState = LWIS_STOP_SCANNING;
			Update->timeMS = 100;
			return LWELR_SLEEP;
		}
		else if (lw20->initState == LWIS_STOP_SCANNING)
		{
			lw20->initState = LWIS_WAIT_STOP_SCANNING;
			packetClear(packet);
			packetWriteString(packet, "#SS,0\r");
			return LWELR_SEND;
		}
		else if (lw20->initState == LWIS_WAIT_STOP_SCANNING)
		{
			lw20->initState = LWIS_GET_PRODUCT;
			return LWELR_AGAIN;
		}
		else if (lw20->initState == LWIS_GET_PRODUCT)
		{
			lw20->initState = LWIS_SENT_GET_PRODUCT;
			packetClear(packet);
			packetWriteString(packet, "?\r");
			return LWELR_SEND;
		}
		else if (lw20->initState == LWIS_SENT_GET_PRODUCT)		
		{
			lw20->initState = LWIS_WAIT_GET_PRODUCT;
			return LWELR_GET_PACKET;
		}
		else if (lw20->initState == LWIS_WAIT_GET_PRODUCT)
		{
			if (Update->responsePacket->type == LWC_PRODUCT)
			{
				// TODO: Add back in.
				//memcpy(&lw20->product, &Update->responsePacket->product, sizeof(lwProductInfo));
				lw20->initState = LWIS_INITED;
				lw20->sendCommand = LWC_NONE;
				return LWELR_INITED;
			}

			// TODO: Keep asking for a packet until timeout expires, then retry for X times, then error out.
			
			return LWELR_GET_PACKET;
		}
	}	

	return LWELR_AGAIN;
}

struct lwResolvePacketResult
{
	lwResolvePacketStatus status;
	int32_t bytesRead;
};

lwResolvePacketResult lw20ResolvePacket(lwResponsePacket* Packet, uint8_t* Buffer, int32_t BufferSize)
{
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
					//printf("Packet Resolve: Bad packet parse\n");
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
				//printf("Packet Resolve: Overflow\n");
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

#endif