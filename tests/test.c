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

    assert(cpu.cycles == 2);
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

void test_adc_zero_x() {
    LOG("running test_adc_zero_x");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // read from third memory addr, test wrapping
    uint8_t program[3] = {0x75, 0xFF, 69};
    cpu.x = 3;

    load_program(&cpu, program, 3);

    execute(&cpu);

    assert(cpu.a == 69);
    assert(cpu.cycles == 4);
    assert(cpu.p == 0);

    destroy_cpu(&cpu);
    LOG("test_adc_zero_x successfull");
}

void test_adc_abs() {
    LOG("running test_adc_abs");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // read from fourth memory addr
    // 			       lo hi
    uint8_t program[4] = {0x6D, 3, 0, 69};

    load_program(&cpu, program, 4);

    execute(&cpu);

    assert(cpu.a == 69);
    assert(cpu.cycles == 4);
    assert(cpu.p == 0);

    destroy_cpu(&cpu);
    LOG("test_adc_abs successfull");
}

void test_adc_abs_x() {
    LOG("running test_adc_abs_x");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // read from second memory addr
    uint8_t program[258]; // 0xff + 3
    memset(program, 0, 258);

    program[0] = 0x7D;
    program[1] = 0xFF;
    program[2] = 0;
    program[257] = 69;

    // 255 + 2 = 257
    cpu.x = 2;

    load_program(&cpu, program, 258);

    execute(&cpu);

    // page boundary crossed
    assert(cpu.cycles == 5);
    assert(cpu.a == 69);
    assert(cpu.p == 0);

    destroy_cpu(&cpu);
    LOG("test_adc_abs_x successfull");
}

void test_adc_abs_y() {
    LOG("running test_adc_abs_y");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // read from fourth memory addr
    uint8_t program[4] = {0x79, 2, 0, 69};
    // 2 + 1 = 3
    cpu.y = 1;

    load_program(&cpu, program, 4);

    execute(&cpu);

    assert(cpu.a == 69);
    // no page boundary crossed
    assert(cpu.cycles == 4);
    assert(cpu.p == 0);

    destroy_cpu(&cpu);
    LOG("test_adc_abs_y successfull");
}

void test_adc_pre_ind() {
    LOG("running test_adc_pre_ind");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    uint8_t program[5] = {0x61, 1, 4, 0, 69};
    cpu.x = 1; // 1 + x = 2
	       // $0003 -> $0004

    load_program(&cpu, program, 5);

    execute(&cpu);

    assert(cpu.cycles == 6);
    assert(cpu.a == 69);
    assert(cpu.p == 0);

    destroy_cpu(&cpu);
    LOG("test_adc_pre_ind successfull");
}

void test_adc_post_ind() {
    LOG("running test_adc_post_ind");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    uint8_t program[5] = {0x71, 2, 1, 0, 69};
    cpu.y = 3; // $0002 -> 1
	       // 1 + y = $0004

    load_program(&cpu, program, 5);

    execute(&cpu);

    // no page boundary
    assert(cpu.cycles == 5);
    assert(cpu.a == 69);
    assert(cpu.p == 0);

    destroy_cpu(&cpu);
    LOG("test_adc_post successfull");
}

void test_asl_acc() {
    LOG("running test_asl_acc");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    uint8_t program[1] = {0x0A};
    cpu.a = 127;

    load_program(&cpu, program, 1);

    execute(&cpu);

    assert(cpu.cycles == 2);
    assert(cpu.a == -2);
    assert(cpu.p == FLAG_N);

    destroy_cpu(&cpu);
    LOG("test_asl_acc successfull");
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
    assert(cpu.p == (FLAG_V | FLAG_N));

    destroy_cpu(&cpu);
    LOG("test_v_flag successfull");
}

void test_branch_extra_cycle() {
    LOG("running test_branch_extra_cycle");
    cpu_t cpu;
    init_cpu(&cpu, 0xFFFF);

    // take branch beq (FLAG_Z is set) to adc (0x69)
    uint8_t program2[6] = {0xF0, 0x02, 0x00, 0x00, 0x69, 80};
    cpu.p |= FLAG_Z;
    assert(cpu.p == FLAG_Z);

    load_program(&cpu, program2, 6);

    execute(&cpu);
    assert(cpu.pc == 4);
    // 2 + 1 (branch taken, no page boundary crossed)
    assert(cpu.cycles == 3);

    // execute adc
    execute(&cpu);

    assert(cpu.a == 80);
    assert(cpu.cycles == 5);

    destroy_cpu(&cpu);
    LOG("test_branch_extra_cycle successfull");
}

int main() {
    test_adc_immediate();
    test_adc_zeropage();
    test_v_flag();
    test_adc_zero_x();
    test_adc_abs();
    test_adc_abs_x();
    test_adc_abs_y();
    test_adc_pre_ind();
    test_adc_post_ind();
    test_asl_acc();
    test_branch_extra_cycle();
    return 0;
}
