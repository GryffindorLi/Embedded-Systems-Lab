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

uint8_t safeuint8pint8(uint8_t a, int8_t b) {
	if (b < 0) {
		if (a < (uint8_t) (-b)) return 0;
		else return a - (uint8_t) (-b);
	} else {
		if ((uint8_t) b > 255 - a) return 255;
		else return a + (uint8_t) b;
	}
}

int8_t safeint8pint8(int8_t a, int8_t b) {
	if (b > 0) {
		if (a > 0 && b > 127 - a) return 127;
		else return a + b;
	} else {
		if (a < 0 && b < -128 - a) return -128;
		else return a + b;
	}
}

int8_t remap1628(int16_t a) {
	return (int8_t) (a / 256);
}

uint8_t remapu1628(uint16_t a) {
	return (uint8_t) (a / 256);
}

int16_t remap8216(int8_t a) {
	return (int16_t) a * 256;
}

uint16_t remapu8216(uint8_t a) {
	return (uint16_t) a * 256;
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

/*
 * @Author Kenrick Trip
 * @Param motor value outputted by the controller.
 * @Return the square root of this value.
 */
int16_t int16sqrt2(int16_t motor_val){
	int16_t k = 1;
	int16_t square = 1;
    int sign = 1;

    if (motor_val < 0) {
        motor_val = -motor_val;
        sign = -1;
    }
	
    if (motor_val == 0 || motor_val == 1)
		return sign*motor_val;
	else {
		while (square <= motor_val) {
			k++;
			square = k*k;
		}
		return sign*(k - 1);
	}
}

uint16_t uint16clamp(uint16_t a, uint16_t min, uint16_t max) {
    return MIN(max, MAX(min, a));
}

int16_t int16clamp(int16_t a, int16_t min, int16_t max) {
    return MIN(max, MAX(min, a));
}

uint8_t uint8clamp(uint8_t a, uint8_t min, uint8_t max) {
    return MIN(max, MAX(min, a));
}

int8_t int8clamp(int8_t a, int8_t min, int8_t max) {
    return MIN(max, MAX(min, a));
}

int32_t log1000(int32_t a) {
	// a = a*1000;
	return (-1742 + ((2821 + ((-1470 + ((447 - (56.57 * a)/1000)) * a)/1000) * a)/1000) * a);
}

