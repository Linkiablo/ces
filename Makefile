
CC := gcc

CFLAGS := -Wall -Wextra -Wpedantic -ggdb3 -fsanitize=address -fsanitize=leak -Wno-unused-parameter -DDEBUG
LIBS := 

OBJS := main.o cpu.o

TEST_OBJS := cpu.o tests/test.o
ROM_TEST_OBJS := cpu.o tests/rom_test.o

all: ces test rom_test

ces: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

test: $(TEST_OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

rom_test: $(ROM_TEST_OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

%/%.o: %/%.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm *.o ces test rom_test tests/*.o

.PHONY:
	all clean test rom_test
