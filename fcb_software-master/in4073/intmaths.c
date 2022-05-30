#include "intmaths.h"

uint16_t safeuint16pint16(uint16_t a, int16_t b) {
	if (b < 0) {
		if (a < (uint16_t) (-b)) return 0;
		else return a - (uint16_t) (-b);
	} else {
		if ((uint16_t) b > 65535 - a) return 65535;
		else return a + (uint16_t) b;
	}
}

int16_t safeint16pint16(int16_t a, int16_t b) {
	if (b > 0) {
		if (a > 0 && b > 32767 - a) return 32767;
		else return a + b;
	} else {
		if (a < 0 && b < -32768 - a) return -32768;
		else return a + b;
	}
}

int16_t int16sqrt(int16_t a) {
    if (a >= 32761) return 181;
    
    int16_t l = 0;
    int16_t r = 180;
    int16_t m = 90;
    int16_t ms = 8100;
    while (l <= r) {
        m = l + (r - l) / 2;
        ms = m * m;
        if (ms <= a) {
            if (a < (m + 1) * (m + 1)) return m;
            l = m + 1;
        } else r = m - 1;
    }
    return 0;
}

uint16_t uint16clamp(uint16_t a, uint16_t min, uint16_t max) {
    return MIN(max, MAX(min, a));
}

int16_t int16clamp(int16_t a, int16_t min, int16_t max) {
    return MIN(max, MAX(min, a));
}


