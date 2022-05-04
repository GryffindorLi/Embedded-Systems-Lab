/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>

#define TRANSMISSION_FREQ 50

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;

void term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void term_puts(char *s)
{
	fprintf(stderr,"%s",s);
}

void term_putchar(char c)
{
	putc(c,stderr);
}

int	term_getchar_nb()
{
	static unsigned char 	line [2];

	if (read(0,line,1)) // note: destructive read
		return (int) line[0];

	return -1;
}

int	term_getchar()
{
	int    c;

	while ((c = term_getchar_nb()) == -1)
		;
	return c;
}

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */
// #include <sys/ioctl.h>
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include "../communication/PC2D.h"
#include "../communication/D2PC.h"
#include "./joystick/joystick.h"

static int fd_serial_port;
/*
 * Open the terminal I/O interface to the serial/pseudo serial port.
 *
 */
void serial_port_open(const char *serial_device)
{
	char *name;
	int result;
	struct termios tty;

	fd_serial_port = open(serial_device, O_RDWR | O_NOCTTY);

	assert(fd_serial_port>=0);

	result = isatty(fd_serial_port);
	assert(result == 1);

	name = ttyname(fd_serial_port);
	assert(name != 0);

	result = tcgetattr(fd_serial_port, &tty);
	assert(result == 0);

	tty.c_iflag = IGNBRK; /* ignore break condition */
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
	tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 1; // TODO check cpu usage

	tty.c_iflag &= ~(IXON|IXOFF|IXANY);

	result = tcsetattr (fd_serial_port, TCSANOW, &tty); /* non-canonical */

	tcflush(fd_serial_port, TCIOFLUSH); /* flush I/O buffer */
}


void serial_port_close(void)
{
	int 	result;

	result = close(fd_serial_port);
	assert (result==0);
}


uint8_t serial_port_getchar()
{
	int8_t result;
	uint8_t c;

	result = read(fd_serial_port, &c, 1);
	if (result == 1) return c;
	else return -1;
}

/*
 * @Author Zirui Li
 * @Param bytes A double pointer to a bytes array. Data read into this array.
 * @Return A flag indicates fail(-1) or succeed(10)
 */
int8_t serial_port_getmessage(uint8_t** bytes){
	int8_t flag;
	do {
		flag = read(fd_serial_port, *bytes, 10);
	} while (flag != 10 && flag != -1);

	return flag;
}

/*
 * @Author: Hanyuan Ban
 * @Param msg The message that needs to be sent..
 * @Return A flag indicates fail(-1) or succeed(sizeof(msg))
 */
int serial_port_putmessage(pc_msg* msg, int len)
{
	int result;
	do {
		result = (int) write(fd_serial_port, msg, len);
	} while (result == 0);

	return result;
}

int send_ctrl_msg(pc_msg* msg, controls cont, char c) {
	msg->cm.checksum = sizeof(msg->cm);
	msg->cm.key = c;
	msg->cm.control = cont;
	
	int bytes = serial_port_putmessage(msg, sizeof(msg->cm));
	if (bytes == -1) {
		fprintf(stderr,"Failed to send from PC to DRONE\n");
	}
	return bytes;
}

int send_mode_msg(pc_msg* msg, uint8_t mode) {
	msg->mm.checksum = sizeof(msg->mm);
	msg->mm.mode = mode;
	
	int bytes = serial_port_putmessage(msg, sizeof(msg->mm));
	if (bytes == -1) {
		fprintf(stderr,"Failed to send from PC to DRONE\n");
	}
	return bytes;
}

uint8_t get_mode_change(char key, int* buttons) {
	if (key == 27) return MODE_PANIC;	//escape
	if (key >= '0' && key <= '8') return (uint8_t) key - '0';  //change mode from
	if (buttons[0] == 1) return MODE_PANIC;
	return 255;
}

void set_controls(controls* cont, int* axis) {
	cont->roll = axis[ROLL_AXIS];
	cont->pitch = axis[PITCH_AXIS];
	cont->yaw = axis[YAW_AXIS];
	cont->throttle = -axis[THROTTLE_AXIS] + 32768;
}

float time_dif(struct timeval st, struct timeval ed) {
	return (ed.tv_sec - st.tv_sec) * 1000.0f + (ed.tv_usec - st.tv_usec) / 1000.0f;
}


/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
 /*
 * @Author Hanyuan Ban
 * @Author Zirui Li
 */
int main(int argc, char **argv)
{	
	// ----------------------------------INITIALIZATION----------------------------------------

	term_initio();
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");
	int fd;
	if ((fd = js_init()) == 0) {
		term_puts("\n Joystick unplugged\n");
	} else {
		term_puts("\n Joystick plugged\n");
	}

	// if no argument is given at execution time, /dev/ttyUSB0 is assumed
	// asserts are in the function
	if (argc == 1) {
		serial_port_open("/dev/ttyUSB0");
	} else if (argc == 2) {
		serial_port_open(argv[1]);
	} else {
		printf("wrong number of arguments\n");
		return -1;
	}

	term_puts("Type ^C to exit\n");

	// ------------------------------------MAIN LOOP------------------------------------------
	struct timeval start;
	struct timeval end;
	int timer_flag = 0;
	char rc = -1;
	char c = -1;
	char tmp_c = -1;
	uint8_t current_mode = MODE_SAFE;
	uint8_t tmp_mode = -1;
	controls cont = {500, 20000, 19999, 19998};
	int axis[6] = {0};
	int buttons[12] = {0};
	JS_message js_msg;
	struct js_event js;
	

	for (;;) {
		if (fd < 0) {
			fprintf(stderr,"\n Joystick unplugged\n");
		}
		if (timer_flag == 0) {
			gettimeofday(&start, 0);
			timer_flag = 1;
		}
		// read the keyboard command every loop
		c = -1;
		if ((tmp_c = term_getchar_nb()) != -1) {
			c = tmp_c;
		}

		// js_flag = 1;
		// if (read_file(fd, js, axis, buttons) == 0) {
		// 	js_flag = 0;
		// }
		read_file(fd, js, axis, buttons);

		create_message_js2D(&js_msg, axis, buttons);
		set_controls(&cont, axis);
		
		tmp_mode = get_mode_change(c, buttons);
		// transmit mode change signal immediately after detection
		if (tmp_mode != 255) {
			current_mode = tmp_mode;
			pc_msg  msg;
			msg.mm = new_mode_msg();
			send_mode_msg(&msg, current_mode);
		}

		// transmit control signal at transmission frequency (50Hz)

		gettimeofday(&end, 0);
		if (time_dif(start, end) > (float) (1000 / TRANSMISSION_FREQ)) {
			timer_flag = 0;
			pc_msg msg;
			msg.cm = new_ctrl_msg();
			send_ctrl_msg(&msg, cont, c);
		}
		
		// receive bytes from drone
		// uint8_t* mess;
		// if ((serial_port_getmessage(&mess)) != -1){
		// 	bytes_array ba;
		// 	memcpy((void*)(&ba.bytes), (void*)mess, 10);

		// 	D2PC_message_p recv_mess = &ba.m;
		// 	printf("Mode is %d\n", recv_mess->mode);
		// 	printf("Battery is %d\n", recv_mess->battery);
		// 	printf("Yaw is %d\n", recv_mess->y);
		// 	printf("Pitch is %d\n", recv_mess->p);
		// 	printf("Roll is %d\n", recv_mess->r);
		// }

		if ((rc = serial_port_getchar()) != -1) {
			term_putchar(rc);
		}
	}

	term_exitio();
	serial_port_close();
	term_puts("\n<exit>\n");
}

