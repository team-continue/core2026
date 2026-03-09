
#include "task_delay.h"
#include <Arduino.h>

void task_delay(uint32_t time_us){
   delayMicroseconds(time_us);
}