#ifndef CPU_H
#define CPU_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLAG_N 128 // 0b10000000
#define FLAG_V 64  // 0b01000000
#define FLAG_B 16  // 0b00010000
#define FLAG_D 8   // 0b00001000
#define FLAG_I 4   // 0b00000100
#define FLAG_Z 2   // 0b00000010
#define FLAG_C 1   // 0b00000001

/* main CPU struct contains registers */
typedef struct cpu_t {
    uint16_t pc;
    uint8_t sp;

    /* status flags */
    uint8_t p;

    /* signed for arithmetic operations */
    int8_t a;

    uint8_t x;
    uint8_t y;

    uint8_t *mem;
    size_t mem_size;

    size_t cycles;
} cpu_t;

int execute(cpu_t *cpu);
void init_cpu(cpu_t *cpu, size_t mem_size);
void load_program(cpu_t *cpu, uint8_t *program, size_t program_size);
void destroy_cpu(cpu_t *cpu);

#endif /* CPU_H */
