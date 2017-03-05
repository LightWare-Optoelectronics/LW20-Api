#include <iostream>

#define LW20_API_IMPLEMENTATION
#include "..\\lw20api.h"

// Linux & Windows usage code is very similar, only coms, time, differences.

struct lwSensorContext
{
	lwLW20		lw20;
	uint8_t		inputBuffer[128];
	int32_t		inputBufferSize;
};

int main()
{
	std::cout << "LW20 Api";

	lwSensorContext context = {};

	context.lw20 = lw20CreateLW20();

	std::cin.ignore();

	return 0;
}