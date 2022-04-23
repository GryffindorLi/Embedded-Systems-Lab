CC=gcc
CFLAGS = -g -Wextra -Wall -Werror -lm
EXEC = ./pc-terminal

default: pc_terminal.o PC2D.o
	$(CC) $(CFLAGS) pc_terminal.c PC2D.o -o $(EXEC)

pc_terminal.o: pc_terminal.c PC2D.h
	cc -c pc_terminal.c

PC2D.o: PC2D.c
	cc -c PC2D.c

clean:
	rm $(EXEC)

run: default
	$(EXEC)
