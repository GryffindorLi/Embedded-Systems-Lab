
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "joystick.h"

/* current axis and button readings
 */
int	axis[6];
int button[12];

// @Author : Karan Pathak
int js_init() {
	int fd;
	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		return -1;
	}
	fcntl(fd, F_SETFL, O_NONBLOCK);
	return fd;
}

// @Author : Karan Pathak
int read_file(int fd, struct js_event js, int* axis, int* button) {
	int result;
	result = read(fd, &js, sizeof(struct js_event));
	if(result == sizeof(struct js_event)) {
		switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				axis[js.number] = js.value;
				break;
			default: break;
		}
	}
	return result;
}
