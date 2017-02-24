//-------------------------------------------------------------------------
// LightWare LW20 Arudino Sample
//-------------------------------------------------------------------------

// TOOD: get around doing this. (Will happen with better com backend implementation)
#define LW20_SERIAL Serial1
#define LW20_API_IMPLEMENTATION
#include "lw20api.h"

lwLW20 lw20;

unsigned long _t;

//-------------------------------------------------------------------------
// Init.
//-------------------------------------------------------------------------
void setup()
{
  Serial.begin(57600);
  while (!Serial);
  
  LW20_SERIAL.begin(115200);
  while (!LW20_SERIAL);

  if (lw20Init(&lw20))
  {
    Serial.print("LW20 ");
    Serial.print(lw20.model);
    Serial.print(" ");
    Serial.print(lw20.firmwareVersion);
    Serial.print(" ");
    Serial.print(lw20.softwareVersion);
    Serial.print(" at ");
    
    int32_t baudRate;
    lw20GetComsBaudRate(&lw20, &baudRate);
    Serial.println(baudRate);

    //lw20SetComsBaudRate(&lw20, LWBR_230400);
    //lw20SaveAll(&lw20);
  }
  else
  {
    Serial.println("LW20 Failed to initialize");
  }

  _t = micros();
}

//-------------------------------------------------------------------------
// Primary Update.
//-------------------------------------------------------------------------

void loop() 
{
  unsigned long dt = micros() - _t;
  _t = micros();
  
  float distance;
  lw20GetDistance(&lw20, LWPT_FIRST, LWRF_RAW, &distance);
  Serial.print(dt);
  Serial.print(' ');
  //Serial.print(" Distance: ");
  Serial.println(distance);

  //delay(100);
}
