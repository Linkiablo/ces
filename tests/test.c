#include "../cpu.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define LOG(s) printf("INFO: %s\n", s)

void test_adc_immediate() {
    LOG("running test_adc_immediate");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // adc immediate A: 0 + 1 = 1
    uint8_t program[2] = {0x69, 0x01};

    load_program(&cpu, program, 2);

    execute(&cpu);

    assert(cpu.cycles == 0);
    assert(cpu.a == 1);
    assert(cpu.p == 0);

    // adc immediate A: 1 + -1 = 0
    uint8_t program2[2] = {0x69, -1};

    load_program(&cpu, program2, 2);

    cpu.pc = 0;

    execute(&cpu);
    assert(cpu.a == 0);
    // zero flag
    assert(cpu.p == FLAG_Z);

    // adc immediate A: 0 + -1 = -1
    uint8_t program3[2] = {0x69, -1};
    load_program(&cpu, program3, 2);

    cpu.pc = 0;

    execute(&cpu);
    assert(cpu.a == -1);
    // zero and carry flag
    assert(cpu.p == (FLAG_N | FLAG_C));

    destroy_cpu(&cpu);
    LOG("test_adc_immediate successfull");
}

void test_adc_zeropage() {
    LOG("running test_adc_zeropage");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // read from second memory addr
    uint8_t program[3] = {0x65, 2, 69};

    load_program(&cpu, program, 3);

    execute(&cpu);

    assert(cpu.a == 69);
    assert(cpu.p == 0);

    destroy_cpu(&cpu);
    LOG("test_adc_zeropage successfull");
}

void test_v_flag() {
    LOG("running test_v_flag");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // adc immediate A: 0 + 80 = 80
    uint8_t program[2] = {0x69, 80};

    load_program(&cpu, program, 2);

    execute(&cpu);

    assert(cpu.a == 80);
    assert(cpu.p == 0);

    // adc immediate A: 80 + 80 = 160 (signed -96)
    uint8_t program2[2] = {0x69, 80};

    load_program(&cpu, program2, 2);

    cpu.pc = 0;

    execute(&cpu);
    assert(cpu.a == -96);
    // overflow flag
    assert(cpu.p == (FLAG_V));

    destroy_cpu(&cpu);
    LOG("test_v_flag successfull");
}

int main() {
    test_adc_immediate();
    test_adc_zeropage();
    test_v_flag();
    return 0;
}
