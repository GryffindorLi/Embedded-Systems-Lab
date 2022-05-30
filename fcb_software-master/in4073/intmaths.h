#include <inttypes.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define SIGN(x, y) (((x) < (y)) ? -1 : 1)

uint16_t safeuint16pint16(uint16_t a, int16_t b);
int16_t safeint16pint16(int16_t a, int16_t b);
int16_t int16sqrt(int16_t a);
uint16_t uint16clamp(uint16_t a, uint16_t min, uint16_t max);
int16_t int16clamp(int16_t a, int16_t min, int16_t max);