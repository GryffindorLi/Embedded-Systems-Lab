#ifndef BAROMETER_H_
#define BAROMETER_H_

#include <inttypes.h>

#define CONVERT_D1_1024 0x44
#define CONVERT_D1_4096 0x48
#define CONVERT_D2_1024 0x54
#define CONVERT_D2_4096 0x58
#define MS5611_ADDR	0b01110111
#define READ		0x0
#define PROM		0xA0

extern int32_t pressure;
extern int32_t temperature;

void read_baro(void);
void baro_init(void);

#endif /* BAROMETER_H_ */
