#include "../cpu.h"
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define LOG(s) printf("INFO: %s\n", s)

void test_rom() {
    LOG("running test_adc_immediate");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    int fd = open("programs/6502_65C02_functional_tests/bin_files/"
                  "6502_functional_test.bin",
                  O_RDWR);
    if(fd == -1){
	    printf("ERROR: %s\n", strerror(errno));
    }

    unsigned char prg[0xFFFF];

    int bytes_read = read(fd, prg, 0xFFFF);
    if(bytes_read == -1){
	    printf("ERROR: %s\n", strerror(errno));
    }
    assert(bytes_read == 0xFFFF);

    load_program(&cpu, prg, 0xFFFF);

    cpu.pc = 0x400;

    while (1) {
        execute(&cpu);
    }

    destroy_cpu(&cpu);
    LOG("test_adc_immediate successfull");
}

int main() {
    test_rom();
    return 0;
}
