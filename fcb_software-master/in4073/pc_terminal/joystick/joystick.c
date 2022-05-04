
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


// int main ()
// {
// 	unsigned int timeout = 300;
// 	int 		fd;
// 	struct js_event js;
// 	JS_message message;
// 	unsigned int	t, i;
// 	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
// 		perror("jstest");
// 		exit(1);
// 	}

// 	/* non-blocking mode
// 	 */
// 	fcntl(fd, F_SETFL, O_NONBLOCK);
// 	//printf("%ld", sizeof(struct js_event));
// 	while (1) {
		
// 		/* simulate work
// 		 */
// 		mon_delay_ms(300);
// 		t = mon_time_ms();
		
// 		while (read(fd, &js, sizeof(struct js_event)) == 
// 		       			sizeof(struct js_event))  {

// 			/* register data
// 			 */
// 			//fprintf(stderr,".");
// 			switch(js.type & ~JS_EVENT_INIT) {
// 				case JS_EVENT_BUTTON:
// 					button[js.number] = js.value;
// 					break;
// 				case JS_EVENT_AXIS:
// 					axis[js.number] = js.value;
// 					break;
// 			}
		
// 		i = mon_time_ms();
// 		if ((t-i)> timeout)
// 		{
// 			printf("Timeout");
// 		}
// 		if (sizeof(struct js_event) != CHECK_SUM)
// 		{
// 			printf("Message Lost");
// 		}

// 		create_message_js2D(&message, axis, button);
// 		//free(&message);
// 		}
		
// 		if (button[1])
// 		{
// 			//Break and send the information to the drone
// 			printf("PANIC: DO NOT PANIC!");
// 			break;

// 		}
// 		if (errno != EAGAIN) {
// 			perror("\njs: error reading (EAGAIN)");//If USB connection of Joystick is lost, 
// 													//Enter safe mode
// 			button[0]=1;
// 			exit (1);
// 		}

// 		printf("\n");
// 		printf("%5d   ",t);
// 		for (i = 0; i < 6; i++) {
// 			printf("%6d ",axis[i]);
// 		}
// 		printf(" |  ");
// 		for (i = 0; i < 12; i++) {
// 			printf("%d ",button[i]);
// 		}
// 		if (button[0])
// 		{
// 			printf("SAFE MODE ENTERED");
// 			break;
// 		}

// 	}
// 	printf("\n<exit>\n");

// }