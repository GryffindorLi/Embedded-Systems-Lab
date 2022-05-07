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
#include <stdlib.h>

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;

void	term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void	term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void	term_puts(char *s)
{
	fprintf(stderr,"%s",s);
}

void	term_putchar(char c)
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
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include "../PC2D.h"
#include "../D2PC.h"

static int is_string = 0;

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

	do {
		result = read(fd_serial_port, &c, 1);
	} while (result != 1);

	return c;
}

/*
 * @Author Zirui Li
 * @Param bytes A double pointer to a bytes array. Data read into this array.
 * @Return A flag indicates fail(-1) or succeed(10)
 */
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

int serial_port_putmessage(PC2D_message mes)
{
	int result;
	do {
		result = (int) write(fd_serial_port, &mes, sizeof(mes));
	} while (result == 0);

	return result;
}

/*
 * @Author Zirui Li
 * @Param mess A byte array holding bytes received from drone.
 * Decode the message according to different header.
 * @Return A void pointer which could later be casted to D2PC_message_p or D2PC_string_message_p.
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
		memcpy((void*)(&(sba.bytes)), (void*)(&mess), sizeof(D2PC_string_message));
		D2PC_string_message_p recv_mess = &(sba.sm);

		return (void*)recv_mess;

	} else {
		printf("No header matched\n");
		return NULL;
	}
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
	char c;

	term_initio();
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

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

	/* send & receive
	 */
	int seq_no = 0;
	for (;;) {
		if ((c = term_getchar_nb()) != -1) {
			seq_no++;
			PC2D_message new_message = create_message();

			set_checksum(&new_message, 10);
			set_mode(&new_message, MODE_SAFE);
			controls cont = {20000,19999,19998};
			set_control(&new_message, cont);
			set_key(&new_message, c);
			
			int bytes = serial_port_putmessage(new_message);
			fprintf(stderr,"Sent %d bytes to DRONE!", bytes);
		}

		/*
		if ((c = serial_port_getchar()) != -1) {
			term_putchar(c);
		}
		*/

		
		uint8_t mess[257];
		if ((serial_port_getmessage(mess)) != -1){
			if (is_string){
				D2PC_string_message_p recv_mess = (D2PC_string_message_p)decode(mess);
				printf("String is %s\n", recv_mess->string);
				free(recv_mess);
			} else {
				D2PC_message_p recv_mess = (D2PC_message_p)decode(mess);
				printf("Mode is %u\n", recv_mess->mode);
				printf("Battery is %u\n", recv_mess->battery);
				printf("Yaw is %d\n", recv_mess->y);
				printf("Pitch is %d\n", recv_mess->p);
				printf("Roll is %d\n", recv_mess->r);
				printf("Motor1 is %u\n", recv_mess->motor1);
				printf("Motor2 is %u\n", recv_mess->motor2);
				printf("Motor3 is %u\n", recv_mess->motor3);
				printf("Motor4 is %u\n", recv_mess->motor4);
				printf("Filtered yaw is %d\n", recv_mess->filtered_y);
				printf("Filtered pitch is %d\n", recv_mess->filtered_p);
				printf("Filtered roll is %d\n", recv_mess->filtered_r);
				free(recv_mess);
			}
			
		}
		
	}

	term_exitio();
	serial_port_close();
	term_puts("\n<exit>\n");

	return 0;
}

