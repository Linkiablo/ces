#ifndef CPU_H
#define CPU_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* main CPU struct contains registers */
typedef struct cpu_t {
    uint16_t pc;
    uint8_t sp;
    /* status flags */
    uint8_t p;
    uint8_t a;
    uint8_t x;
    uint8_t y;

    uint8_t *mem;
    size_t mem_size;

    size_t cycles;
} cpu_t;

int start(cpu_t *cpu);
void init_cpu(cpu_t *cpu, size_t mem_size);
void destroy_cpu(cpu_t *cpu);

#endif /* CPU_H */
