CC=gcc
LD=gcc
CFLAGS = -g -Wextra -Wall -Werror
LDFLAGS = -lm
EXEC = ./pc-terminal

default: pc_terminal.o PC2D.o D2PC.o
	$(LD) $(LDFLAGS) pc_terminal.o PC2D.o D2PC.o -o $(EXEC)

pc_terminal.o: pc_terminal.c
	$(CC) $(CFLAGS) -c pc_terminal.c -o pc_terminal.o

PC2D.o: ../communication/PC2D.c
	$(CC) $(CFLAGS) -c ../communication/PC2D.c -o PC2D.o

D2PC.o: ../communication/D2PC.c
	$(CC) $(CFLAGS) -c ../communication/D2PC.c -o D2PC.o

clean:
	rm $(EXEC)
	rm *.o

run: default
	$(EXEC)
