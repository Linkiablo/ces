
CC := gcc

CFLAGS := -Wall -Wextra -Wpedantic -ggdb -fsanitize=address -fsanitize=leak -Wno-unused-parameter
LIBS := 

OBJS := main.o cpu.o

TEST_OBJS := cpu.o tests/test.o

all: ces test

ces: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

test: $(TEST_OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

%/%.o: %/%.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm *.o ces test tests/*.o

.PHONY:
	all clean test
