
CC := gcc

CFLAGS := -Wall -Wextra -Wpedantic -ggdb
LIBS := 

OBJS := main.o cpu.o

all: ces

ces: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm *.o ces

.PHONY:
	all clean
