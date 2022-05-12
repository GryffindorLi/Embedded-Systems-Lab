#include <stdbool.h>
#include "PC2D.h"

CTRL_msg new_ctrl_msg() {
    CTRL_msg new_message = {'C', 't', {0, 0, 0, 0}, '\0', 0};
    new_message.checksum = sizeof(new_message);
    return new_message;
}

MODE_msg new_mode_msg() {
    MODE_msg new_message = {'M', 'd', MODE_SAFE};
    return new_message;
}

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