CC=gcc
LD=gcc
CFLAGS = -g -Wextra -Wall -Werror
LDFLAGS = -lm
EXEC = ./pc-terminal

default: pc_terminal.o PC2D.o D2PC.o logging.o
	$(LD) $(LDFLAGS) pc_terminal.o logging.o PC2D.o D2PC.o -o $(EXEC)

pc_terminal.o: pc_terminal.c
	$(CC) $(CFLAGS) -c pc_terminal.c -o pc_terminal.o

PC2D.o: ../PC2D.c
	$(CC) $(CFLAGS) -c ../PC2D.c -o PC2D.o

D2PC.o: ../D2PC.c
	$(CC) $(CFLAGS) -c ../D2PC.c -o D2PC.o

logging.o: logging.c
	$(CC) $(CFLAGS) -c ./logging.c -o logging.o

clean:
	rm $(EXEC)
	rm *.o

run: default
	$(EXEC)
