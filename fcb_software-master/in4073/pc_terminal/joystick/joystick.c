
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
#include "../../communication/PC2D.h"

/* current axis and button readings
 */
int	axis[6];
int button[12];

void create_message_js2D(JS_message* message, int* axis, int* button)
{
	//message = (new_message*) malloc (sizeof(new_message));
	for (int i=0;i<6;i++)
	{
		message->axis[i]= axis[i];
	}
	for (int j=0;j<12;j++)
	{
		message->button[j]=button[j];
	}
}
/* time
 */
#include <time.h>
#include <assert.h>
unsigned int    mon_time_ms(void)
{
        unsigned int    ms;
        struct timeval  tv;
        struct timezone tz;

        gettimeofday(&tv, &tz);
        ms = 1000 * (tv.tv_sec % 65); // 65 sec wrap around
        ms = ms + tv.tv_usec / 1000;
        return ms;
}

void    mon_delay_ms(unsigned int ms)
{
        struct timespec req, rem;

        req.tv_sec = ms / 1000;
        req.tv_nsec = 1000000 * (ms % 1000);
        assert(nanosleep(&req,&rem) == 0);
}

int js_init() {
	int fd;
	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		return 0;
	}
	fcntl(fd, F_SETFL, O_NONBLOCK);
	return fd;
}

void read_file(int fd, struct js_event js, int* axis, int* button) {
	if(read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {
		switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				axis[js.number] = js.value;
				break;
		}
	}
}