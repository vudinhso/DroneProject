#include <Arduino.h>

void init_serial()
{
  Serial.begin(115200);
  while (!Serial)
    ;
}
