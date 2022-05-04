CC=gcc
LD = gcc
UTILS = /utils
CFLAGS = -g -Wextra -Wall -Werror
LDFLAGS = -lm

EXEC = ./pc-terminal

default: pc_terminal.o PC2D.o ./joystick/joystick.o
	$(LD) $(LDFLAGS) pc_terminal.o PC2D.o ./joystick/joystick.o -o $(EXEC)

pc_terminal.o: pc_terminal.c ../communication/PC2D.h ./joystick/joystick.h
	$(CC) $(CFLAGS) -c pc_terminal.c 

<<<<<<< HEAD
joystick.o: ./joystick/joystick.c
	$(CC) $(CFLAGS) -c ./joystick/joystick.c

PC2D.o: ../communication/PC2D.c
	$(CC) $(CFLAGS) -c ../communication/PC2D.c
=======
pc_terminal.o: pc_terminal.c PC2D.h
	cc -c pc_terminal.c

PC2D.o: PC2D.c
	cc -c PC2D.c
>>>>>>> 44f5bee0fd596a0d03cded2577400856a9511881

clean:
	rm $(EXEC)

run: default
	$(EXEC)
