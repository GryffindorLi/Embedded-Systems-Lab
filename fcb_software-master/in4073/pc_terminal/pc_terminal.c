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
#include "../communication/PC2D.h"
#include "../communication/D2PC.h"

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
int serial_port_putmessage(pc_msg msg, int len)
{
	int result;
	do {
		result = (int) write(fd_serial_port, &msg, len);
	} while (result == 0);

	return result;
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

	/* 
	 *send & receive
	 */

	clock_t time = 0;
	char c = -1;
	char tmp_c = -1;
	// uint8_t current_mode = MODE_SAFE;
	controls cont = {20000, 19999, 19998};
	// uint8_t buttons[12] = {};

	for (;;) {
		// read the keyboard command every loop
		if ((tmp_c = term_getchar_nb()) != -1) {
			c = tmp_c;
		}




		// --------------------------------------------------
		/*
			TODO: read joystick

				get xyz controls into:
					cont = {uint16_t, uint16_t, uint16_t};

				(uncomment the buttons[12] on top of for(;;))
				
				get buttons into:    
					buttons = [uint8_t * 12];
				
		*/
		// --------------------------------------------------




		// transmit control signal at transmission frequency (50Hz)
		if (clock() - time > TRANSMISSION_FREQ) {
			time = clock();
			pc_msg msg;
			msg.cm = new_ctrl_msg();
			msg.cm.checksum = sizeof(msg.cm);
			msg.cm.key = c;
			msg.cm.control = cont;
			
			int bytes = serial_port_putmessage(msg, sizeof(msg.cm));
			if (bytes > -1) {
				fprintf(stderr,"Sent %d bytes to DRONE!", bytes);
			} else {
				fprintf(stderr,"Failed to send from PC to DRONE");
			}
		}
		
		// receive bytes from drone
		uint8_t* mess;
		if ((serial_port_getmessage(&mess)) != -1){
			bytes_array ba;
			memcpy((void*)(&ba.bytes), (void*)mess, 10);

			D2PC_message_p recv_mess = &ba.m;
			printf("Mode is %d\n", recv_mess->mode);
			printf("Battery is %d\n", recv_mess->battery);
			printf("Yaw is %d\n", recv_mess->y);
			printf("Pitch is %d\n", recv_mess->p);
			printf("Roll is %d\n", recv_mess->r);
		}
	}

	term_exitio();
	serial_port_close();
	term_puts("\n<exit>\n");
}

