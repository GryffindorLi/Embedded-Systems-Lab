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
	static unsigned char c1[2];
	static unsigned char c2[2];
	static unsigned char c3[2];

	if (read(0,c1,1)) {
		if (c1[0] == 27) {
			if (read(0,c2,1)) {
				if (c2[0] == 91) {
					if (read(0,c3,1)) {
						if (c3[0] == 65) return '!'; //up
						else if (c3[0] == 66) return '@'; //down
						else if (c3[0] == 67) return '#'; //right
						else if (c3[0] == 68) return '$'; //left
					}
				}
			}
		}
		return c1[0];
	}
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
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include "../D2PC.h"
#include "joystick.h"
#include <stdlib.h>
#include "../PC2D.h"

static int fd_serial_port;
static int is_string = 0;
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

int16_t serial_port_getmessage(uint8_t bytes[]){
	int16_t size = 0;
	int8_t flag;
	while ((flag = read(fd_serial_port, &bytes[size], 1)) != -1){
		if (bytes[size] == TAIL){
			break;
		}
		size++;
	}
	is_string = (bytes[0] == STRING_HEADER)? 1: 0;
	return (flag == -1)? flag: size;
}

/*
 * @Author Zirui Li
 * @Param bytes A double pointer to a char array. Data read into this array.
 * @Return The number of 
 * Deprecated!!! Use serial_port_getmessage instead.
 */
int16_t serial_port_getstring(char string[]){
	int16_t size = 0;
	int8_t flag;
	while ((flag = read(fd_serial_port, &string[size], 1)) != -1){
		if (string[size] == TAIL){
			break;
		}
		size++;
	}
	return (flag == -1)? flag: size;
}

/*
 * @Author Zirui Li
 * @Param mess A byte array holding bytes received from drone.
 * @Param recv_mess A pointer to a D2PC_message struct
 */
void* decode(uint8_t mess[]) {
	if (!is_string){
		D2PC_message_p recv_mess = (D2PC_message_p)malloc(sizeof(D2PC_message));
		recv_mess->mode = mess[1];
		recv_mess->battery = mess[2];
		recv_mess->y = ((int16_t)(mess[3]) << 8) + (int16_t)mess[4];
		recv_mess->p = ((int16_t)(mess[5]) << 8) + (int16_t)mess[6];
		recv_mess->r = ((int16_t)(mess[7]) << 8) + (int16_t)mess[8];
		recv_mess->filtered_y = ((int16_t)(mess[9]) << 8) + (int16_t)mess[10];
		recv_mess->filtered_p = ((int16_t)(mess[11]) << 8) + (int16_t)mess[12];
		recv_mess->filtered_r = ((int16_t)(mess[13]) << 8) + (int16_t)mess[14];
		recv_mess->motor1 = ((uint16_t)(mess[15]) << 8) + (uint16_t)mess[16];
		recv_mess->motor2 = ((uint16_t)(mess[17]) << 8) + (uint16_t)mess[18];
		recv_mess->motor3 = ((uint16_t)(mess[19]) << 8) + (uint16_t)mess[20];
		recv_mess->motor4 = ((uint16_t)(mess[21]) << 8) + (uint16_t)mess[22];
		recv_mess->checksum = ((uint16_t)(mess[23]) << 8) + (uint16_t)mess[24];

		return (void*)recv_mess;

	} else if (is_string) {
		string_bytes_array sba;
		memcpy((void*)(&(sba.bytes)), (void*)(&mess[1]), sizeof(D2PC_string_message));
		D2PC_string_message_p recv_mess = 
			(D2PC_string_message_p)malloc(sizeof(D2PC_string_message));
		memcpy((void*)recv_mess, (void*)(&(sba.sm)), sizeof(D2PC_string_message));

		return (void*)recv_mess;

	} else {
		printf("No header matched\n");
		return NULL;
	}
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

uint8_t get_mode_change(char key, controls cont, int* buttons) {
	if (key == 27) return MODE_PANIC;	//escape
	if (key == '0') return 0;
	if (key == '1' || buttons[0] == 1) return MODE_PANIC;
	if (cont.pitch == 0 && cont.roll == 0 && cont.yaw == 0 && cont.throttle == 0 ) {
		if (key >= '2' && key <= '8') return (uint8_t) key - '0';  //change mode from
	}
	return 255;
}

void set_controls(controls* cont, int* axis) {
	cont->roll = axis[ROLL_AXIS];
	cont->pitch = axis[PITCH_AXIS];
	cont->yaw = axis[YAW_AXIS];
	// cont->throttle = -axis[THROTTLE_AXIS] + 32767;
	cont->throttle = axis[THROTTLE_AXIS];
}

float time_dif(struct timeval st, struct timeval ed) {
	return (ed.tv_sec - st.tv_sec) * 1000.0f + (ed.tv_usec - st.tv_usec) / 1000.0f;
}

#define TRANSMISSION_FREQ 20
#define JOYSTICK_WATCHDOG_LIFETIME 200

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */

int main(int argc, char **argv)
{	
	// ----------------------------------INITIALIZATION----------------------------------------

	term_initio();
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");
	int fd;
	if ((fd = js_init()) == -1) {
		term_puts("\nJoystick unplugged\n");
	} else {
		term_puts("\nJoystick plugged\n");
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
	struct timeval ctrl_trans_start;
	struct timeval ctrl_trans_end;
	// struct timeval loop_monitor_start;
	// struct timeval loop_monitor_end;
	int joystick_watchdog = JOYSTICK_WATCHDOG_LIFETIME;
	char rc = -1;
	char c = -1;
	char tmp_c = -1;
	uint8_t current_mode = MODE_SAFE;
	uint8_t tmp_mode = -1;
	controls cont = {0, 0, 0, 0};
	int axis[6] = {0};
	int buttons[12] = {0};
	struct js_event js;
	
	gettimeofday(&ctrl_trans_start, 0);
	for (;;) {
		// read the keyboard command every loop
		if ((tmp_c = term_getchar_nb()) != -1) {
			c = tmp_c;
		}

		// read controls and detect connection of joystick
		if (read_file(fd, js, axis, buttons) == -1) {
			joystick_watchdog -= 1;
			if (joystick_watchdog < 0) {
				term_puts("\nJOYSTICK UNPLUGGED\n");
				joystick_watchdog = JOYSTICK_WATCHDOG_LIFETIME;
			}
		} 

		// update controls
		set_controls(&cont, axis);
		
		tmp_mode = get_mode_change(c, cont, buttons);
		// transmit mode change signal immediately after detection
		if (tmp_mode != 255) {
			current_mode = tmp_mode;
			pc_msg  msg;
			msg.mm = new_mode_msg();
			send_mode_msg(&msg, current_mode);
		}
		

		// transmit control signal at transmission frequency (50Hz)
		gettimeofday(&ctrl_trans_end, 0);
		if (time_dif(ctrl_trans_start, ctrl_trans_end) > (float) (1000 / TRANSMISSION_FREQ)) {
			gettimeofday(&ctrl_trans_start, 0);
			pc_msg msg;
			msg.cm = new_ctrl_msg();
			send_ctrl_msg(&msg, cont, c);
			c = -1; // reset key
		}

		if ((rc = serial_port_getchar()) != -1) {
			term_putchar(rc);
		}
	}

	term_exitio();
	serial_port_close();
	term_puts("\n<exit>\n");
}

