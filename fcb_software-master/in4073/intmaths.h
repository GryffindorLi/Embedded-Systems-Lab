#include <inttypes.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define SIGN(x, y) (((x) < (y)) ? -1 : 1)

uint16_t safeuint16pint16(uint16_t a, int16_t b);
int16_t safeint16pint16(int16_t a, int16_t b);
uint8_t safeuint8pint8(uint8_t a, int8_t b);
int8_t safeint8pint8(int8_t a, int8_t b);

int16_t int16sqrt(int16_t a);

int8_t remap1628(int16_t a);
uint8_t remapu1628(uint16_t a);
int16_t remap8216(int8_t a);
uint16_t remapu8216(uint8_t a);

uint16_t uint16clamp(uint16_t a, uint16_t min, uint16_t max);
int16_t int16clamp(int16_t a, int16_t min, int16_t max);
uint8_t uint8clamp(uint8_t a, uint8_t min, uint8_t max);
int8_t int8clamp(int8_t a, int8_t min, int8_t max);
int32_t log1000(int32_t a);
int32_t arccos164(int16_t a);
