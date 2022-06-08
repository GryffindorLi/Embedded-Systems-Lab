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
#include "../config.h"
#include "../D2PC.h"

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

#ifdef LOG_FROM_TERMINAL
void file_putchar(char c, FILE* fp)
{
	putc(c, fp);
}
#endif

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
#include "joystick.h"
#include <stdlib.h>
#include "../PC2D.h"
#include "../control.h"
#include "../keyboard.h"
#include "logging.h"
#include "../intmaths.h"

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

	fcntl(fd_serial_port, F_SETFL, O_NONBLOCK); // set unblocking

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
 * Decode the message according to different header.
 * @Return A void pointer which could later be casted to D2PC_message_p or D2PC_string_message_p.
 */

void* decode(uint8_t mess[], int16_t start) {
	//int start = 0;
	if (!is_string){
		D2PC_message_p recv_mess = (D2PC_message_p)malloc(sizeof(D2PC_message));
		recv_mess->mode = mess[start + 1];
		//recv_mess->battery = mess[start + 2];
		
		// recv_mess->y = (((int32_t)(mess[3])) << 24) + (((int32_t)(mess[4])) << 16) +
		// 	(((int32_t)(mess[5])) << 8) + (int32_t)mess[6];
		// recv_mess->p = (((int32_t)(mess[7])) << 24) + (((int32_t)(mess[8])) << 16) +
		// 	(((int32_t)(mess[9])) << 8) + (int32_t)mess[10];
		// recv_mess->r = (((int32_t)(mess[11])) << 24) + (((int32_t)(mess[12])) << 16) +
		// 	(((int32_t)(mess[13])) << 8) + (int32_t)mess[14];
		// recv_mess->filtered_y = (((int16_t)(mess[15])) << 8) + (int16_t)mess[16];
		// recv_mess->filtered_p = (((int16_t)(mess[17])) << 8) + (int16_t)mess[18];
		// recv_mess->filtered_r = (((int16_t)(mess[19])) << 8) + (int16_t)mess[20];
		// recv_mess->motor1 = (((uint16_t)(mess[21])) << 8) + (uint16_t)mess[22];
		// recv_mess->motor2 = (((uint16_t)(mess[23])) << 8) + (uint16_t)mess[24];
		// recv_mess->motor3 = (((uint16_t)(mess[25])) << 8) + (uint16_t)mess[26];
		// recv_mess->motor4 = (((uint16_t)(mess[27])) << 8) + (uint16_t)mess[28];
		// recv_mess->checksum = (((uint16_t)(mess[29])) << 8) + (uint16_t)mess[30];
		
		recv_mess->y = combine32Byte(mess[start+2], mess[start+3], mess[start+4], mess[start+5]);
		recv_mess->p = combine32Byte(mess[start+6], mess[start+7], mess[start+8], mess[start+9]);
		recv_mess->r = combine32Byte(mess[start+10], mess[start+11], mess[start+12], mess[start+13]);
		recv_mess->filtered_y = combineByte(mess[start+14], mess[start+15]);
		recv_mess->filtered_p = combineByte(mess[start+16], mess[start+17]);
		recv_mess->filtered_r = combineByte(mess[start+18], mess[start+19]);
		recv_mess->motor1 = combineByte(mess[start+20], mess[start+21]);
		recv_mess->motor2 = combineByte(mess[start+22], mess[start+23]);
		recv_mess->motor3 = combineByte(mess[start+24], mess[start+25]);
		recv_mess->motor4 = combineByte(mess[start+26], mess[start+27]);
		recv_mess->checksum = combineByte(mess[start+28], mess[start+29]);
		recv_mess->ts = combine32UByte(mess[30], mess[31], mess[32], mess[33]);
		//recv_mess->timestamp = combine32UByte(mess[31], mess[32], mess[33], mess[34]);

		return (void*)recv_mess;

	} else if (is_string) {
		string_bytes_array sba;
		memcpy((void*)(&(sba.bytes)), (void*)(&mess[0]), sizeof(D2PC_string_message));
		D2PC_string_message_p recv_mess = 
			(D2PC_string_message_p)malloc(sizeof(D2PC_string_message));
		memcpy((void*)recv_mess, (void*)(&(sba.sm)), sizeof(D2PC_string_message));

		return (void*)recv_mess;

	} else {
		printf("No header matched\n");
		return NULL;
	}
}


int send_ctrl_msg(controls cont, char c) {
	CTRL_msg msg = new_ctrl_msg();
	msg.checksum = sizeof(CTRL_msg);
	msg.key = c;
	msg.control = cont;
	int bytes;
	do {
		bytes = (int) write(fd_serial_port, &msg, sizeof(CTRL_msg));
	} while (bytes == 0);
	if (bytes == -1) {
		fprintf(stderr,"Failed to send from PC to DRONE\n");
	}
	return bytes;
}

int send_mode_msg(uint8_t mode) {
	MODE_msg msg = new_mode_msg();
	msg.mode = mode;
	int bytes;
	do {
		bytes = (int) write(fd_serial_port, &msg, sizeof(MODE_msg));
	} while (bytes == 0);
	if (bytes == -1) {
		fprintf(stderr,"Failed to send from PC to DRONE\n");
	}
	return bytes;
}

uint8_t get_mode_change(char key, controls cont, int* buttons, uint8_t current_mode) {
	if (key == 27) return MODE_PANIC;	//escape
	if (key == '0') return 0;
	if (key == '1' || buttons[0] == 1) return MODE_PANIC;
	if (key >= '2' && key <= '8') {
		if (cont.pitch == 0 && cont.roll == 0 && cont.yaw == 0 && cont.throttle == 0 ) 
			return (uint8_t) key - '0';
		else if (current_mode == MODE_FULL_CONTROL && key == '7')
			return (uint8_t) key - '0';
		else if (current_mode == MODE_HEIGHT_CONTROL && key == '5')
			return (uint8_t) key - '0';
		else
			fprintf(stderr,"Keep Controls Neutral!!\n");
	}
	return 255;
}

void joystick_control(controls* cont, int* axis) {
	cont->roll = axis[ROLL_AXIS];
	cont->pitch = -axis[PITCH_AXIS];
	cont->yaw = axis[YAW_AXIS];
	cont->throttle = -axis[THROTTLE_AXIS] + 32767;
}

int keyboard_control(controls* cont, char c) {
	switch (c) {
		case 32:	// space
			cont->throttle = safeuint16pint16(cont->throttle, 100);
			break;
		case 'x':
			cont->throttle = safeuint16pint16(cont->throttle, -100);

			break;
		case 't':
			cont->pitch = safeint16pint16(cont->pitch, 100);
			break;
		case 'g':
			cont->pitch = safeint16pint16(cont->pitch, -100);
			break;
		case 'f':
			cont->roll = safeint16pint16(cont->roll, 100);
			break;
		case 'h':
			cont->roll = safeint16pint16(cont->roll, -100);
			break;
		case 'r':
			cont->yaw = safeint16pint16(cont->yaw, 100);
			break;
		case 'y':
			cont->yaw = safeint16pint16(cont->yaw, -100);
			break;
		case 'c':	// clear
			cont->throttle = 0;
			cont->pitch = 0;
			cont->roll = 0;
			cont->yaw = 0;
			break;
		default:
			return 0;
			break;
	}
	return 1;
}


float time_dif(struct timeval st, struct timeval ed) {
	return (ed.tv_sec - st.tv_sec) * 1000.0f + (ed.tv_usec - st.tv_usec) / 1000.0f;
}


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

#ifdef LOG_FROM_TERMINAL
	char FileName[256];
	time_t curr = time(NULL);

	char* prefix = "logs_";
	strcpy(FileName, prefix);
	strcat(FileName, ctime(&curr));

	FILE* fp = fopen(FileName, "a");
#endif

	// ------------------------------------MAIN LOOP------------------------------------------
	struct timeval ctrl_trans_start;
	struct timeval ctrl_trans_end;
	#ifndef JOYSTICK
		// struct timeval ctrl_monitor_start;
		// struct timeval ctrl_monitor_end;
		// gettimeofday(&ctrl_monitor_start, 0);
	#else
		int joystick_watchdog = JOYSTICK_WATCHDOG_LIFETIME;
		struct js_event js;
		int axis[6] = {0};
	#endif

	char rc = -1;
	char c = -1;
	char tmp_c = -1;
	uint8_t current_mode = MODE_SAFE;
	uint8_t tmp_mode = -1;
	controls cont = {0, 0, 0, 0, 0};
	int buttons[12] = {0};
	
	gettimeofday(&ctrl_trans_start, 0);
	for (;;) {

		// read the keyboard command every loop
		if ((tmp_c = term_getchar_nb()) != -1) {
			c = tmp_c;
		}
		
		#ifndef JOYSTICK
			if (keyboard_control(&cont, c)) c = -1;
		#else
			// read controls and detect connection of joystick
			if (read_file(fd, js, axis, buttons) == -1) {
				joystick_watchdog -= 1;
				if (joystick_watchdog < 0) {
					//term_puts("\nJOYSTICK UNPLUGGED\n");
					joystick_watchdog = JOYSTICK_WATCHDOG_LIFETIME;
				}
			} 

			joystick_control(&cont, axis);
		#endif


		tmp_mode = get_mode_change(c, cont, buttons, current_mode);
		// transmit mode change signal immediately after detection
		if (tmp_mode != 255) {
			current_mode = tmp_mode;
			send_mode_msg(current_mode);
			c = -1;
		}
		
		// transmit control signal at transmission frequency
		gettimeofday(&ctrl_trans_end, 0);
		if (time_dif(ctrl_trans_start, ctrl_trans_end)*1000 > (1000000 / TRANSMISSION_FREQ)) {
			gettimeofday(&ctrl_trans_start, 0);
			send_ctrl_msg(cont, c);
			c = -1; // reset key
		}
#ifdef LOG_FROM_TERMINAL
		if ((rc = serial_port_getchar()) != -1) {
			term_putchar(rc);
			file_putchar(rc, fp);
		}
#endif

#ifndef LOG_FROM_TERMINAL	
		uint8_t mess[500];
		int16_t start;
		if ((start = serial_port_getmessage(mess)) != -1){
			//printf("The start position is %d\n", start);
			if (is_string){
				D2PC_string_message_p recv_mess = (D2PC_string_message_p)decode(mess, 0);
				printf("String is %s\n", recv_mess->string);
				logging((void*)recv_mess, is_string);
				free(recv_mess);
			} else {
				D2PC_message_p recv_mess = (D2PC_message_p)decode(mess, 0);
				uint16_t recv_cs = cal_checksum(*recv_mess);
				if (recv_cs == recv_mess->checksum){
					logging((void*)recv_mess, is_string);
					//print_D2PC_message(recv_mess);
					printf("The index is %hhu\n", recv_mess->idx);
				} else {
					perror("Message disrupted during transmission!");
				}
				free(recv_mess);
			}
			
		}
#endif
	}
#ifdef LOG_FROM_TERMINAL
	fclose(fp);
#endif
	term_exitio();
	serial_port_close();
	term_puts("\n<exit>\n");
}

