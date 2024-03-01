#ifndef __DS1307_H__
#define __DS1307_H__

#include <time.h>

#define SDA_PIN 18
#define SCL_PIN 19
#define DS1307_ADDRESS 0x68

void initDS1307(void);
void writeValue(time_t newTime);
time_t readValue();

#endif
