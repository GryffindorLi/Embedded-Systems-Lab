CC=gcc
LD = gcc
UTILS = /utils
CFLAGS = -g -Wextra -Wall -Werror
LDFLAGS = -lm

EXEC = ./pc-terminal

default: pc_terminal.o PC2D.o joystick.o
	$(LD) $(LDFLAGS) pc_terminal.o PC2D.o joystick.o -o $(EXEC)

pc_terminal.o: pc_terminal.c ../PC2D.h joystick.h
	$(CC) $(CFLAGS) -c pc_terminal.c 

joystick.o: joystick.c
	$(CC) $(CFLAGS) -c joystick.c

PC2D.o: ../PC2D.c
	$(CC) $(CFLAGS) -c ../PC2D.c

clean:
	rm $(EXEC)

run: default
	$(EXEC)
