#include "../cpu.h"
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define LOG(s) printf("INFO: %s\n", s)

void test_rom() {
    LOG("running test_rom");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    int fd = open("programs/6502_65C02_functional_tests/bin_files/"
                  "6502_functional_test.bin",
                  O_RDWR);
    // int fd = open("programs/ft.bin", O_RDWR);
    if (fd == -1) {
        printf("ERROR: %s\n", strerror(errno));
    }

    unsigned char prg[0xFFFF];

    int bytes_read = read(fd, prg, 0xFFFF);
    if (bytes_read == -1) {
        printf("ERROR: %s\n", strerror(errno));
    }

    load_program(&cpu, prg, 0xFFFF);

    cpu.pc = 0x400;

    while (1) {
        uint16_t prev_pc = cpu.pc;
        execute(&cpu);
        if (cpu.pc == prev_pc) {
            fprintf(stderr, "ERROR: loop on PC detected! aborting...\n");
            exit(1);
        }
    }

    destroy_cpu(&cpu);
    LOG("test_rom successfull");
}

int main() {
    test_rom();
    return 0;
}
