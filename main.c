#include "cpu.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

int main() {

    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // adc immediate A: 0 + 1 = 1
    uint8_t program[2] = { 0x69, 0x01 };
    uint8_t *pp = (uint8_t *) malloc(2);
    memcpy(pp, program, 2);

    load_program(&cpu, pp, 2);

    execute(&cpu);

    assert(cpu.a == 1);
    assert(cpu.p == 0);

    // adc immediate A: 0 + 1 = 1
    uint8_t program2[2] = { 0x69, -1 };
    uint8_t *pp2 = (uint8_t *) malloc(2);
    memcpy(pp2, program2, 2);

    load_program(&cpu, pp2, 2);

    cpu.pc = 0;

    execute(&cpu);
    assert(cpu.a == 0);
    assert(cpu.p == FLAG_Z);

    uint8_t program3[2] = { 0x69, -1 };
    uint8_t *pp3 = (uint8_t *) malloc(2);
    memcpy(pp3, program3, 2);

    load_program(&cpu, pp3, 2);

    cpu.pc = 0;

    execute(&cpu);
    assert(cpu.a == -1);
    assert(cpu.p == (FLAG_N | FLAG_C));

    return 0;
}
