#include <LW20.h>

LW20 lw20(Serial1, 115200);

void setup() 
{
  // Start serial monitor port.
  Serial.begin(115200);

  // Setup LW20.
  lw20.init();
  lw20.setLaserParams(LW20_MODE_388);
  lw20.setServoParams(1000, 2000, 10);
  lw20.setScanParams(-45, 45, 8, 3.5);
  lw20.setAlarmAParams(0.5, -45, -20);
  lw20.setAlarmBParams(0.5, 20, 45);
  lw20.startScan();
}

void loop() 
{
  // Check alarm status.
  bool a, b;
  lw20.checkAlarmStatus(&a, &b);

  Serial.print("Alarm A:");
  Serial.print(a);
  Serial.print(" Alarm B:");
  Serial.println(b);
  
  delay(100);
}
