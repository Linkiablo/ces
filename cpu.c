#include "cpu.h"
#include <assert.h>
#include <stdbool.h>

#define READ_8(cpu) (_read_8(cpu))
#define READ_16(cpu) (READ_8(cpu) | (READ_8(cpu) << 8))

#define LOAD_8(addr) (*((uint8_t *)addr))
#define LOAD_16(addr) (LOAD_8(addr) | (LOAD_8(addr + 1) << 8))

#define SET_FLAG(cpu, flag, cond)                                              \
    (cond ? (cpu->p |= flag) : (cpu->p &= ~(flag)))

#define CHECK_FLAG_N(cpu, res) SET_FLAG(cpu, FLAG_N, ((res & 128) != 0))
#define CHECK_FLAG_C(cpu, res) SET_FLAG(cpu, FLAG_C, ((res & 0xFF00) != 0))
// don't account for carry / overflow
#define CHECK_FLAG_Z(cpu, res) SET_FLAG(cpu, FLAG_Z, ((res & 0xFF) == 0))
// v flag:
// https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
// M = *oper, N = cpu->a
#define CHECK_FLAG_V(cpu, m, n, res)                                           \
    SET_FLAG(cpu, FLAG_V, ((m ^ res) & (n ^ res) & 0x80) != 0)

#define INSTRUCTION(i, m, c, cb)                                               \
    { .inst = i, .mode = m, .cycles = c, .check_boundary = cb }

/* private method because of possible undefined order of operations */
static uint8_t _read_8(cpu_t *cpu) { return cpu->mem[cpu->pc++]; }

void init_cpu(cpu_t *cpu, size_t mem_size) {
    memset(cpu, 0, sizeof(cpu_t));

    // TODO: min size?
    /* PC 16 bit wide */
    assert(mem_size <= UINT16_MAX);
    cpu->mem_size = mem_size;

    cpu->mem = (uint8_t *)malloc(mem_size);
    if (cpu->mem == NULL) {
        fprintf(stderr, "ERROR: could not allocate %ld bytes for cpu memory!",
                mem_size);
    }

    // TODO: stack pointer init
}

void destroy_cpu(cpu_t *cpu) { free(cpu->mem); }

void load_program(cpu_t *cpu, uint8_t *program, size_t program_size) {
    memcpy(cpu->mem, program, program_size);
}

enum address_mode {
    ACCUMULATOR,
    IMMEDIATE,
    ABSOLUTE,
    ZERO_PAGE,
    ABS_X,
    ABS_Y,
    ZERO_X,
    ZERO_Y,
    INDIRECT,
    PRE_INDIRECT,
    POST_INDIRECT,
    IMPLIED,
    RELATIVE
};

static uint8_t *get_oper_ptr(cpu_t *cpu, enum address_mode mode,
                             bool check_boundary) {
    uint16_t offset = 0;

    switch (mode) {
    case ACCUMULATOR:
        // early return
        return (uint8_t *)&(cpu->a);
    case IMMEDIATE:
        // increment PC
        READ_8(cpu);
        // -1 because pc points at next instruction
        offset = cpu->pc - 1;
        break;
    case ABSOLUTE:
        offset = READ_16(cpu);
        break;
    case ZERO_PAGE:
        offset = READ_8(cpu);
        break;
    case ABS_X:
        offset = READ_16(cpu) + cpu->x;
        break;
    case ABS_Y:
        offset = READ_16(cpu) + cpu->y;
        break;
    case ZERO_X:;
        // never affects hi byte, instead wrapping add
        offset = (uint8_t)(READ_8(cpu) + cpu->x);
        break;
    case ZERO_Y:;
        offset = (uint8_t)(READ_8(cpu) + cpu->y);
        break;
    case INDIRECT:;
        void *abs_addr = cpu->mem + READ_16(cpu);
        offset = LOAD_16(abs_addr);
        break;
    case PRE_INDIRECT:;                           // empty statement
        uint8_t offset_ix = READ_8(cpu) + cpu->x; // wrapping add
        void *xz_addr = cpu->mem + offset_ix;     // X_ZERO
        offset = LOAD_16(xz_addr);
        break;
    case POST_INDIRECT:;                        // empty statement
        void *yz_addr = cpu->mem + READ_8(cpu); // Y_ZERO
        offset = LOAD_16(yz_addr) + cpu->y;
        break;
    case RELATIVE:;
        int8_t offset_rel = READ_8(cpu);
        offset = cpu->pc + offset_rel;
        break;
    default:
        fprintf(stderr, "ERROR: unknown address mode!");
        exit(1);
    }

    if (check_boundary) {
        // add cycle if page boundary is crossed
        if ((offset >> 8) != (cpu->pc >> 8)) {
            cpu->cycles += 1;
        }
    }

    return cpu->mem + offset;
}

typedef void (*instruction_fn)(cpu_t *cpu, uint8_t *oper_ptr);

typedef struct instruction_t {
    instruction_fn inst;
    enum address_mode mode;
    uint8_t cycles;
    bool check_boundary;
} instruction_t;

// ADC
//
//     Add Memory to Accumulator with Carry
//
//     A + M + C -> A, C
//     N	Z	C	I	D	V
//     +	+	+	-	-	+
//     addressing	assembler	opc	bytes	cycles
//     immediate	ADC #oper	69	2	2
//     zeropage		ADC oper	65	2	3
//     zeropage,X	ADC oper,X	75	2	4
//     absolute		ADC oper	6D	3	4
//     absolute,X	ADC oper,X	7D	3	4*
//     absolute,Y	ADC oper,Y	79	3	4*
//     (indirect,X)	ADC (oper,X)	61	2	6
//     (indirect),Y	ADC (oper),Y	71	2	5*
void adc(cpu_t *cpu, uint8_t *oper_ptr) {
    // int8_t *oper = (int8_t *)get_oper_ptr(cpu, mode, check_boundary);
    int8_t oper = *oper_ptr;

    // carry works, because carry flag is the first bit
    int16_t res = oper + cpu->a + (cpu->p & FLAG_C);

    CHECK_FLAG_N(cpu, res);
    CHECK_FLAG_Z(cpu, res);
    CHECK_FLAG_C(cpu, res);
    CHECK_FLAG_V(cpu, oper, cpu->a, res);

    cpu->a = res & 0xFF;
}

// AND
//
//     AND Memory with Accumulator
//
//     A AND M -> A
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	AND #oper	29	2	2
//     zeropage		AND oper	25	2	3
//     zeropage,X	AND oper,X	35	2	4
//     absolute		AND oper	2D	3	4
//     absolute,X	AND oper,X	3D	3	4*
//     absolute,Y	AND oper,Y	39	3	4*
//     (indirect,X)	AND (oper,X)	21	2	6
//     (indirect),Y	AND (oper),Y	31	2	5*
void and (cpu_t * cpu, uint8_t *oper_ptr) {
    // TODO: komisches clangd formatting?
    int8_t oper = *oper_ptr;

    int8_t res = oper & cpu->a;
    CHECK_FLAG_N(cpu, res);
    CHECK_FLAG_Z(cpu, res);

    cpu->a = res;
}

// ASL
//
//     Shift Left One Bit (Memory or Accumulator)
//
//     C <- [76543210] <- 0
//     N	Z	C	I	D	V
//     +	+	+	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     accumulator	ASL A		0A	1	2
//     zeropage		ASL oper	06	2	5
//     zeropage,X	ASL oper,X	16	2	6
//     absolute		ASL oper	0E	3	6
//     absolute,X	ASL oper,X	1E	3	7
void asl(cpu_t *cpu, uint8_t *oper_ptr) {
    *oper_ptr <<= 1;

    CHECK_FLAG_N(cpu, *oper_ptr);
    CHECK_FLAG_Z(cpu, *oper_ptr);
    CHECK_FLAG_C(cpu, *oper_ptr);
}

// BCC
//
//     Branch on Carry Clear
//
//     branch on C = 0
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     relative	BCC oper	90	2	2**
void bcc(cpu_t *cpu, uint8_t *oper_ptr) {}

// BCS
//
//     Branch on Carry Set
//
//     branch on C = 1
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     relative	BCS oper	B0	2	2**
void bcs(cpu_t *cpu, uint8_t *oper_ptr) {}

// BEQ
//
//     Branch on Result Zero
//
//     branch on Z = 1
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     relative	BEQ oper	F0	2	2**
void beq(cpu_t *cpu, uint8_t *oper_ptr) {}

// BIT
//
//     Test Bits in Memory with Accumulator
//
//     bits 7 and 6 of operand are transfered to bit 7 and 6 of SR (N,V);
//     the zero-flag is set to the result of operand AND accumulator.
//
//     A AND M, M7 -> N, M6 -> V
//     N	Z	C	I	D	V
//     M7	+	-	-	-	M6
//     addressing	assembler	opc	bytes	cycles
//     zeropage	BIT oper	24	2	3
//     absolute	BIT oper	2C	3	4
void bit(cpu_t *cpu, uint8_t *oper_ptr) {}

// BMI
//
//     Branch on Result Minus
//
//     branch on N = 1
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     relative	BMI oper	30	2	2**
void bmi(cpu_t *cpu, uint8_t *oper_ptr) {}

// BNE
//
//     Branch on Result not Zero
//
//     branch on Z = 0
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     relative	BNE oper	D0	2	2**
void bne(cpu_t *cpu, uint8_t *oper_ptr) {}

// BPL
//
//     Branch on Result Plus
//
//     branch on N = 0
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     relative	BPL oper	10	2	2**
void bpl(cpu_t *cpu, uint8_t *oper_ptr) {}

// BRK
//
//     Force Break
//
//     BRK initiates a software interrupt similar to a hardware
//     interrupt (IRQ). The return address pushed to the stack is
//     PC+2, providing an extra byte of spacing for a break mark
//     (identifying a reason for the break.)
//     The status register will be pushed to the stack with the break
//     flag set to 1. However, when retrieved during RTI or by a PLP
//     instruction, the break flag will be ignored.
//     The interrupt disable flag is not set automatically.
//
//     interrupt,
//     push PC+2, push SR
//     N	Z	C	I	D	V
//     -	-	-	1	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	BRK	00	1	7
void brk(cpu_t *cpu, uint8_t *oper_ptr) {}

// BVC
//
//     Branch on Overflow Clear
//
//     branch on V = 0
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     relative	BVC oper	50	2	2**
void bvc(cpu_t *cpu, uint8_t *oper_ptr) {}

// BVS
//
//     Branch on Overflow Set
//
//     branch on V = 1
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     relative	BVS oper	70	2	2**
void bvs(cpu_t *cpu, uint8_t *oper_ptr) {}

// CLC
//
//     Clear Carry Flag
//
//     0 -> C
//     N	Z	C	I	D	V
//     -	-	0	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	CLC	18	1	2
void clc(cpu_t *cpu, uint8_t *oper_ptr) {}

// CLD
//
//     Clear Decimal Mode
//
//     0 -> D
//     N	Z	C	I	D	V
//     -	-	-	-	0	-
//     addressing	assembler	opc	bytes	cycles
//     implied	CLD	D8	1	2
void cld(cpu_t *cpu, uint8_t *oper_ptr) {}

// CLI
//
//     Clear Interrupt Disable Bit
//
//     0 -> I
//     N	Z	C	I	D	V
//     -	-	-	0	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	CLI	58	1	2
void cli(cpu_t *cpu, uint8_t *oper_ptr) {}

// CLV
//
//     Clear Overflow Flag
//
//     0 -> V
//     N	Z	C	I	D	V
//     -	-	-	-	-	0
//     addressing	assembler	opc	bytes	cycles
//     implied	CLV	B8	1	2
void clv(cpu_t *cpu, uint8_t *oper_ptr) {}

// CMP
//
//     Compare Memory with Accumulator
//
//     A - M
//     N	Z	C	I	D	V
//     +	+	+	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	CMP #oper	C9	2	2
//     zeropage	CMP oper	C5	2	3
//     zeropage,X	CMP oper,X	D5	2	4
//     absolute	CMP oper	CD	3	4
//     absolute,X	CMP oper,X	DD	3	4*
//     absolute,Y	CMP oper,Y	D9	3	4*
//     (indirect,X)	CMP (oper,X)	C1	2	6
//     (indirect),Y	CMP (oper),Y	D1	2	5*
void cmp(cpu_t *cpu, uint8_t *oper_ptr) {}

// CPX
//
//     Compare Memory and Index X
//
//     X - M
//     N	Z	C	I	D	V
//     +	+	+	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	CPX #oper	E0	2	2
//     zeropage	CPX oper	E4	2	3
//     absolute	CPX oper	EC	3	4
void cpx(cpu_t *cpu, uint8_t *oper_ptr) {}

// CPY
//
//     Compare Memory and Index Y
//
//     Y - M
//     N	Z	C	I	D	V
//     +	+	+	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	CPY #oper	C0	2	2
//     zeropage	CPY oper	C4	2	3
//     absolute	CPY oper	CC	3	4
void cpy(cpu_t *cpu, uint8_t *oper_ptr) {}

// DEC
//
//     Decrement Memory by One
//
//     M - 1 -> M
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     zeropage	DEC oper	C6	2	5
//     zeropage,X	DEC oper,X	D6	2	6
//     absolute	DEC oper	CE	3	6
//     absolute,X	DEC oper,X	DE	3	7
void dec(cpu_t *cpu, uint8_t *oper_ptr) {}

// DEX
//
//     Decrement Index X by One
//
//     X - 1 -> X
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	DEX	CA	1	2
void dex(cpu_t *cpu, uint8_t *oper_ptr) {}

// DEY
//
//     Decrement Index Y by One
//
//     Y - 1 -> Y
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	DEY	88	1	2
void dey(cpu_t *cpu, uint8_t *oper_ptr) {}

// EOR
//
//     Exclusive-OR Memory with Accumulator
//
//     A EOR M -> A
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	EOR #oper	49	2	2
//     zeropage	EOR oper	45	2	3
//     zeropage,X	EOR oper,X	55	2	4
//     absolute	EOR oper	4D	3	4
//     absolute,X	EOR oper,X	5D	3	4*
//     absolute,Y	EOR oper,Y	59	3	4*
//     (indirect,X)	EOR (oper,X)	41	2	6
//     (indirect),Y	EOR (oper),Y	51	2	5*
void eor(cpu_t *cpu, uint8_t *oper_ptr) {}

// INC
//
//     Increment Memory by One
//
//     M + 1 -> M
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     zeropage	INC oper	E6	2	5
//     zeropage,X	INC oper,X	F6	2	6
//     absolute	INC oper	EE	3	6
//     absolute,X	INC oper,X	FE	3	7
void inc(cpu_t *cpu, uint8_t *oper_ptr) {}

// INX
//
//     Increment Index X by One
//
//     X + 1 -> X
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	INX	E8	1	2
void inx(cpu_t *cpu, uint8_t *oper_ptr) {}

// INY
//
//     Increment Index Y by One
//
//     Y + 1 -> Y
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	INY	C8	1	2
void iny(cpu_t *cpu, uint8_t *oper_ptr) {}

// JMP
//
//     Jump to New Location
//
//     (PC+1) -> PCL
//     (PC+2) -> PCH
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     absolute	JMP oper	4C	3	3
//     indirect	JMP (oper)	6C	3	5
void jmp(cpu_t *cpu, uint8_t *oper_ptr) {}

// JSR
//
//     Jump to New Location Saving Return Address
//
//     push (PC+2),
//     (PC+1) -> PCL
//     (PC+2) -> PCH
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     absolute	JSR oper	20	3	6
void jsr(cpu_t *cpu, uint8_t *oper_ptr) {}

// LDA
//
//     Load Accumulator with Memory
//
//     M -> A
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	LDA #oper	A9	2	2
//     zeropage	LDA oper	A5	2	3
//     zeropage,X	LDA oper,X	B5	2	4
//     absolute	LDA oper	AD	3	4
//     absolute,X	LDA oper,X	BD	3	4*
//     absolute,Y	LDA oper,Y	B9	3	4*
//     (indirect,X)	LDA (oper,X)	A1	2	6
//     (indirect),Y	LDA (oper),Y	B1	2	5*
void lda(cpu_t *cpu, uint8_t *oper_ptr) {}

// LDX
//
//     Load Index X with Memory
//
//     M -> X
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	LDX #oper	A2	2	2
//     zeropage	LDX oper	A6	2	3
//     zeropage,Y	LDX oper,Y	B6	2	4
//     absolute	LDX oper	AE	3	4
//     absolute,Y	LDX oper,Y	BE	3	4*
void ldx(cpu_t *cpu, uint8_t *oper_ptr) {}

// LDY
//
//     Load Index Y with Memory
//
//     M -> Y
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	LDY #oper	A0	2	2
//     zeropage	LDY oper	A4	2	3
//     zeropage,X	LDY oper,X	B4	2	4
//     absolute	LDY oper	AC	3	4
//     absolute,X	LDY oper,X	BC	3	4*
void ldy(cpu_t *cpu, uint8_t *oper_ptr) {}

// LSR
//
//     Shift One Bit Right (Memory or Accumulator)
//
//     0 -> [76543210] -> C
//     N	Z	C	I	D	V
//     0	+	+	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     accumulator	LSR A	4A	1	2
//     zeropage	LSR oper	46	2	5
//     zeropage,X	LSR oper,X	56	2	6
//     absolute	LSR oper	4E	3	6
//     absolute,X	LSR oper,X	5E	3	7
void lsr(cpu_t *cpu, uint8_t *oper_ptr) {}

// NOP
//
//     No Operation
//
//     ---
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied		NOP		EA	1	2
void nop(cpu_t *cpu, uint8_t *oper_ptr) {}

// ORA
//
//     OR Memory with Accumulator
//
//     A OR M -> A
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     immediate	ORA #oper	09	2	2
//     zeropage	ORA oper	05	2	3
//     zeropage,X	ORA oper,X	15	2	4
//     absolute	ORA oper	0D	3	4
//     absolute,X	ORA oper,X	1D	3	4*
//     absolute,Y	ORA oper,Y	19	3	4*
//     (indirect,X)	ORA (oper,X)	01	2	6
//     (indirect),Y	ORA (oper),Y	11	2	5*
void ora(cpu_t *cpu, uint8_t *oper_ptr) {}

// PHA
//
//     Push Accumulator on Stack
//
//     push A
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	PHA	48	1	3
void pha(cpu_t *cpu, uint8_t *oper_ptr) {}

// PHP
//
//     Push Processor Status on Stack
//
//     The status register will be pushed with the break
//     flag and bit 5 set to 1.
//
//     push SR
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	PHP	08	1	3
void php(cpu_t *cpu, uint8_t *oper_ptr) {}

// PLA
//
//     Pull Accumulator from Stack
//
//     pull A
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	PLA	68	1	4
void pla(cpu_t *cpu, uint8_t *oper_ptr) {}

// PLP
//
//     Pull Processor Status from Stack
//
//     The status register will be pulled with the break
//     flag and bit 5 ignored.
//
//     pull SR
//     N	Z	C	I	D	V
//     from stack
//     addressing	assembler	opc	bytes	cycles
//     implied	PLP	28	1	4
void plp(cpu_t *cpu, uint8_t *oper_ptr) {}

// ROL
//
//     Rotate One Bit Left (Memory or Accumulator)
//
//     C <- [76543210] <- C
//     N	Z	C	I	D	V
//     +	+	+	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     accumulator	ROL A	2A	1	2
//     zeropage	ROL oper	26	2	5
//     zeropage,X	ROL oper,X	36	2	6
//     absolute	ROL oper	2E	3	6
//     absolute,X	ROL oper,X	3E	3	7
void rol(cpu_t *cpu, uint8_t *oper_ptr) {}

// ROR
//
//     Rotate One Bit Right (Memory or Accumulator)
//
//     C -> [76543210] -> C
//     N	Z	C	I	D	V
//     +	+	+	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     accumulator	ROR A	6A	1	2
//     zeropage	ROR oper	66	2	5
//     zeropage,X	ROR oper,X	76	2	6
//     absolute	ROR oper	6E	3	6
//     absolute,X	ROR oper,X	7E	3	7
void ror(cpu_t *cpu, uint8_t *oper_ptr) {}

// RTI
//
//     Return from Interrupt
//
//     The status register is pulled with the break flag
//     and bit 5 ignored. Then PC is pulled from the stack.
//
//     pull SR, pull PC
//     N	Z	C	I	D	V
//     from stack
//     addressing	assembler	opc	bytes	cycles
//     implied	RTI	40	1	6
void rti(cpu_t *cpu, uint8_t *oper_ptr) {}

// RTS
//
//     Return from Subroutine
//
//     pull PC, PC+1 -> PC
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	RTS	60	1	6
void rts(cpu_t *cpu, uint8_t *oper_ptr) {}

// SBC
//
//     Subtract Memory from Accumulator with Borrow
//
//     A - M - C -> A
//     N	Z	C	I	D	V
//     +	+	+	-	-	+
//     addressing	assembler	opc	bytes	cycles
//     immediate	SBC #oper	E9	2	2
//     zeropage	SBC oper	E5	2	3
//     zeropage,X	SBC oper,X	F5	2	4
//     absolute	SBC oper	ED	3	4
//     absolute,X	SBC oper,X	FD	3	4*
//     absolute,Y	SBC oper,Y	F9	3	4*
//     (indirect,X)	SBC (oper,X)	E1	2	6
//     (indirect),Y	SBC (oper),Y	F1	2	5*
void sbc(cpu_t *cpu, uint8_t *oper_ptr) {}

// SEC
//
//     Set Carry Flag
//
//     1 -> C
//     N	Z	C	I	D	V
//     -	-	1	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	SEC	38	1	2
void sec(cpu_t *cpu, uint8_t *oper_ptr) {}

// SED
//
//     Set Decimal Flag
//
//     1 -> D
//     N	Z	C	I	D	V
//     -	-	-	-	1	-
//     addressing	assembler	opc	bytes	cycles
//     implied	SED	F8	1	2
void sed(cpu_t *cpu, uint8_t *oper_ptr) {}

// SEI
//
//     Set Interrupt Disable Status
//
//     1 -> I
//     N	Z	C	I	D	V
//     -	-	-	1	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	SEI	78	1	2
void sei(cpu_t *cpu, uint8_t *oper_ptr) {}

// STA
//
//     Store Accumulator in Memory
//
//     A -> M
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     zeropage	STA oper	85	2	3
//     zeropage,X	STA oper,X	95	2	4
//     absolute	STA oper	8D	3	4
//     absolute,X	STA oper,X	9D	3	5
//     absolute,Y	STA oper,Y	99	3	5
//     (indirect,X)	STA (oper,X)	81	2	6
//     (indirect),Y	STA (oper),Y	91	2	6
void sta(cpu_t *cpu, uint8_t *oper_ptr) {}

// STX
//
//     Store Index X in Memory
//
//     X -> M
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     zeropage	STX oper	86	2	3
//     zeropage,Y	STX oper,Y	96	2	4
//     absolute	STX oper	8E	3	4
void stx(cpu_t *cpu, uint8_t *oper_ptr) {}

// STY
//
//     Sore Index Y in Memory
//
//     Y -> M
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     zeropage	STY oper	84	2	3
//     zeropage,X	STY oper,X	94	2	4
//     absolute	STY oper	8C	3	4
void sty(cpu_t *cpu, uint8_t *oper_ptr) {}

// TAX
//
//     Transfer Accumulator to Index X
//
//     A -> X
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	TAX	AA	1	2
void tax(cpu_t *cpu, uint8_t *oper_ptr) {}

// TAY
//
//     Transfer Accumulator to Index Y
//
//     A -> Y
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	TAY	A8	1	2
void tay(cpu_t *cpu, uint8_t *oper_ptr) {}

// TSX
//
//     Transfer Stack Pointer to Index X
//
//     SP -> X
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	TSX	BA	1	2
void tsx(cpu_t *cpu, uint8_t *oper_ptr) {}

// TXA
//
//     Transfer Index X to Accumulator
//
//     X -> A
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	TXA	8A	1	2
void txa(cpu_t *cpu, uint8_t *oper_ptr) {}

// TXS
//
//     Transfer Index X to Stack Register
//
//     X -> SP
//     N	Z	C	I	D	V
//     -	-	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	TXS	9A	1	2
void txs(cpu_t *cpu, uint8_t *oper_ptr) {}

// TYA
//
//     Transfer Index Y to Accumulator
//
//     Y -> A
//     N	Z	C	I	D	V
//     +	+	-	-	-	-
//     addressing	assembler	opc	bytes	cycles
//     implied	TYA	98	1	82
void tya(cpu_t *cpu, uint8_t *oper_ptr) {}

instruction_t const INSTRUCTION_LOOKUP[0xFF] = {
    INSTRUCTION(brk, IMPLIED, 7, false),       // 0x00
    INSTRUCTION(ora, PRE_INDIRECT, 6, false),  // 0x01
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x02
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x03
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x04
    INSTRUCTION(ora, ZERO_PAGE, 3, false),     // 0x05
    INSTRUCTION(asl, ZERO_PAGE, 5, false),     // 0x06
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x07
    INSTRUCTION(php, IMPLIED, 3, false),       // 0x08
    INSTRUCTION(ora, IMMEDIATE, 2, false),     // 0x09
    INSTRUCTION(asl, ACCUMULATOR, 2, false),   // 0x0A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x0B
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x0C
    INSTRUCTION(ora, ABSOLUTE, 4, false),      // 0x0D
    INSTRUCTION(asl, ABSOLUTE, 6, false),      // 0x0E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x0F
    INSTRUCTION(bpl, RELATIVE, 2, true),       // 0x10
    INSTRUCTION(ora, POST_INDIRECT, 5, true),  // 0x11
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x12
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x13
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x14
    INSTRUCTION(ora, ZERO_X, 4, false),        // 0x15
    INSTRUCTION(asl, ZERO_X, 6, false),        // 0x16
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x17
    INSTRUCTION(clc, IMPLIED, 2, false),       // 0x18
    INSTRUCTION(ora, ABS_Y, 4, true),          // 0x19
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x1A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x1B
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x1C
    INSTRUCTION(ora, ABS_X, 4, true),          // 0x1D
    INSTRUCTION(asl, ABS_X, 7, false),         // 0x1E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x1F
    INSTRUCTION(jsr, ABSOLUTE, 6, false),      // 0x20
    INSTRUCTION(and, PRE_INDIRECT, 6, false),  // 0x21
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x22
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x23
    INSTRUCTION(bit, ZERO_PAGE, 3, false),     // 0x24
    INSTRUCTION(and, ZERO_PAGE, 3, false),     // 0x25
    INSTRUCTION(rol, ZERO_PAGE, 5, false),     // 0x26
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x27
    INSTRUCTION(plp, IMPLIED, 4, false),       // 0x28
    INSTRUCTION(and, IMMEDIATE, 2, false),     // 0x29
    INSTRUCTION(rol, ACCUMULATOR, 2, false),   // 0x2A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x2B
    INSTRUCTION(bit, ABSOLUTE, 4, false),      // 0x2C
    INSTRUCTION(and, ABSOLUTE, 4, false),      // 0x2D
    INSTRUCTION(rol, ABSOLUTE, 6, false),      // 0x2E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x2F
    INSTRUCTION(bmi, RELATIVE, 2, true),       // 0x30
    INSTRUCTION(and, POST_INDIRECT, 5, true),  // 0x31
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x32
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x33
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x34
    INSTRUCTION(and, ZERO_X, 4, false),        // 0x35
    INSTRUCTION(rol, ZERO_X, 6, false),        // 0x36
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x37
    INSTRUCTION(sec, IMPLIED, 2, false),       // 0x38
    INSTRUCTION(and, ABS_Y, 4, true),          // 0x39
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x3A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x3B
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x3C
    INSTRUCTION(and, ABS_X, 4, true),          // 0x3D
    INSTRUCTION(rol, ABS_X, 7, false),         // 0x3E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x3F
    INSTRUCTION(rti, IMPLIED, 6, false),       // 0x40
    INSTRUCTION(eor, PRE_INDIRECT, 6, false),  // 0x41
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x42
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x43
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x44
    INSTRUCTION(eor, ZERO_PAGE, 3, false),     // 0x45
    INSTRUCTION(lsr, ZERO_PAGE, 5, false),     // 0x46
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x47
    INSTRUCTION(pha, IMPLIED, 3, false),       // 0x48
    INSTRUCTION(eor, IMMEDIATE, 2, false),     // 0x49
    INSTRUCTION(lsr, ACCUMULATOR, 2, false),   // 0x4A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x4B
    INSTRUCTION(jmp, ABSOLUTE, 3, false),      // 0x4C
    INSTRUCTION(eor, ABSOLUTE, 4, false),      // 0x4D
    INSTRUCTION(lsr, ABSOLUTE, 6, false),      // 0x4E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x4F
    INSTRUCTION(bvc, RELATIVE, 2, true),       // 0x50
    INSTRUCTION(eor, POST_INDIRECT, 5, true),  // 0x51
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x52
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x53
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x54
    INSTRUCTION(eor, ZERO_X, 4, false),        // 0x55
    INSTRUCTION(lsr, ZERO_X, 6, false),        // 0x56
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x57
    INSTRUCTION(cli, IMPLIED, 2, false),       // 0x58
    INSTRUCTION(eor, ABS_Y, 4, true),          // 0x59
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x5A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x5B
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x5C
    INSTRUCTION(eor, ABS_X, 4, true),          // 0x5D
    INSTRUCTION(lsr, ABS_X, 7, false),         // 0x5E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x5F
    INSTRUCTION(rts, IMPLIED, 6, false),       // 0x60
    INSTRUCTION(adc, PRE_INDIRECT, 6, false),  // 0x61
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x62
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x63
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x64
    INSTRUCTION(adc, ZERO_PAGE, 3, false),     // 0x65
    INSTRUCTION(ror, ZERO_PAGE, 5, false),     // 0x66
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x67
    INSTRUCTION(pla, IMPLIED, 4, false),       // 0x68
    INSTRUCTION(adc, IMMEDIATE, 2, false),     // 0x69
    INSTRUCTION(ror, ACCUMULATOR, 2, false),   // 0x6A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x6B
    INSTRUCTION(jmp, INDIRECT, 5, false),      // 0x6C
    INSTRUCTION(adc, ABSOLUTE, 4, false),      // 0x6D
    INSTRUCTION(ror, ABSOLUTE, 6, false),      // 0x6E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x6F
    INSTRUCTION(bvs, RELATIVE, 2, true),       // 0x70
    INSTRUCTION(adc, POST_INDIRECT, 5, true),  // 0x71
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x72
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x73
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x74
    INSTRUCTION(adc, ZERO_X, 4, false),        // 0x75
    INSTRUCTION(ror, ZERO_X, 6, false),        // 0x76
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x77
    INSTRUCTION(sei, IMPLIED, 2, false),       // 0x78
    INSTRUCTION(adc, ABS_Y, 4, true),          // 0x79
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x7A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x7B
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x7C
    INSTRUCTION(adc, ABS_X, 4, true),          // 0x7D
    INSTRUCTION(ror, ABS_X, 7, false),         // 0x7E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x7F
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x80
    INSTRUCTION(sta, PRE_INDIRECT, 6, false),  // 0x81
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x82
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x83
    INSTRUCTION(sty, ZERO_PAGE, 3, false),     // 0x84
    INSTRUCTION(sta, ZERO_PAGE, 3, false),     // 0x85
    INSTRUCTION(stx, ZERO_PAGE, 3, false),     // 0x86
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x87
    INSTRUCTION(dey, IMPLIED, 2, false),       // 0x88
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x89
    INSTRUCTION(txa, IMPLIED, 2, false),       // 0x8A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x8B
    INSTRUCTION(sty, ABSOLUTE, 4, false),      // 0x8C
    INSTRUCTION(sta, ABSOLUTE, 4, false),      // 0x8D
    INSTRUCTION(stx, ABSOLUTE, 4, false),      // 0x8E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x8F
    INSTRUCTION(bcc, RELATIVE, 2, true),       // 0x90
    INSTRUCTION(sta, POST_INDIRECT, 6, false), // 0x91
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x92
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x93
    INSTRUCTION(sty, ZERO_X, 4, false),        // 0x94
    INSTRUCTION(sta, ZERO_X, 4, false),        // 0x95
    INSTRUCTION(stx, ZERO_Y, 4, false),        // 0x96
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x97
    INSTRUCTION(tya, IMPLIED, 2, false),       // 0x98
    INSTRUCTION(sta, ABS_Y, 5, false),         // 0x99
    INSTRUCTION(txs, IMPLIED, 2, false),       // 0x9A
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x9B
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x9C
    INSTRUCTION(sta, ABS_X, 5, false),         // 0x9D
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x9E
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0x9F
    INSTRUCTION(ldy, IMMEDIATE, 2, false),     // 0xA0
    INSTRUCTION(lda, PRE_INDIRECT, 6, false),  // 0xA1
    INSTRUCTION(ldx, IMMEDIATE, 2, false),     // 0xA2
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xA3
    INSTRUCTION(ldy, ZERO_PAGE, 3, false),     // 0xA4
    INSTRUCTION(lda, ZERO_PAGE, 3, false),     // 0xA5
    INSTRUCTION(ldx, ZERO_PAGE, 3, false),     // 0xA6
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xA7
    INSTRUCTION(tay, IMPLIED, 2, false),       // 0xA8
    INSTRUCTION(lda, IMMEDIATE, 2, false),     // 0xA9
    INSTRUCTION(tax, IMPLIED, 2, false),       // 0xAA
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xAB
    INSTRUCTION(ldy, ABSOLUTE, 4, false),      // 0xAC
    INSTRUCTION(lda, ABSOLUTE, 4, false),      // 0xAD
    INSTRUCTION(ldx, ABSOLUTE, 4, false),      // 0xAE
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xAF
    INSTRUCTION(bcs, RELATIVE, 2, true),       // 0xB0
    INSTRUCTION(lda, POST_INDIRECT, 5, true),  // 0xB1
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xB2
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xB3
    INSTRUCTION(ldy, ZERO_X, 4, false),        // 0xB4
    INSTRUCTION(lda, ZERO_X, 4, false),        // 0xB5
    INSTRUCTION(ldx, ZERO_Y, 4, false),        // 0xB6
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xB7
    INSTRUCTION(clv, IMPLIED, 2, false),       // 0xB8
    INSTRUCTION(lda, ABS_Y, 4, true),          // 0xB9
    INSTRUCTION(tsx, IMPLIED, 2, false),       // 0xBA
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xBB
    INSTRUCTION(ldy, ABS_X, 4, true),          // 0xBC
    INSTRUCTION(lda, ABS_X, 4, true),          // 0xBD
    INSTRUCTION(ldx, ABS_Y, 4, true),          // 0xBE
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xBF
    INSTRUCTION(cpy, IMMEDIATE, 2, false),     // 0xC0
    INSTRUCTION(cmp, PRE_INDIRECT, 6, false),  // 0xC1
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xC2
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xC3
    INSTRUCTION(cpy, ZERO_PAGE, 3, false),     // 0xC4
    INSTRUCTION(cmp, ZERO_PAGE, 3, false),     // 0xC5
    INSTRUCTION(dec, ZERO_PAGE, 5, false),     // 0xC6
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xC7
    INSTRUCTION(iny, IMPLIED, 2, false),       // 0xC8
    INSTRUCTION(cmp, IMMEDIATE, 2, false),     // 0xC9
    INSTRUCTION(dex, IMPLIED, 2, false),       // 0xCA
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xCB
    INSTRUCTION(cpy, ABSOLUTE, 4, false),      // 0xCC
    INSTRUCTION(cmp, ABSOLUTE, 4, false),      // 0xCD
    INSTRUCTION(dec, ABSOLUTE, 6, false),      // 0xCE
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xCF
    INSTRUCTION(bne, RELATIVE, 2, true),       // 0xD0
    INSTRUCTION(cmp, POST_INDIRECT, 5, true),  // 0xD1
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xD2
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xD3
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xD4
    INSTRUCTION(cmp, ZERO_X, 4, false),        // 0xD5
    INSTRUCTION(dec, ZERO_X, 6, false),        // 0xD6
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xD7
    INSTRUCTION(cld, IMPLIED, 2, false),       // 0xD8
    INSTRUCTION(cmp, ABS_Y, 4, true),          // 0xD9
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xDA
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xDB
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xDC
    INSTRUCTION(cmp, ABS_X, 4, true),          // 0xDD
    INSTRUCTION(dec, ABS_X, 7, false),         // 0xDE
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xDF
    INSTRUCTION(cpx, IMMEDIATE, 2, false),     // 0xE0
    INSTRUCTION(sbc, PRE_INDIRECT, 6, false),  // 0xE1
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xE2
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xE3
    INSTRUCTION(cpx, ZERO_PAGE, 3, false),     // 0xE4
    INSTRUCTION(sbc, ZERO_PAGE, 3, false),     // 0xE5
    INSTRUCTION(inc, ZERO_PAGE, 5, false),     // 0xE6
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xE7
    INSTRUCTION(inx, IMPLIED, 2, false),       // 0xE8
    INSTRUCTION(sbc, IMMEDIATE, 2, false),     // 0xE9
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xEA
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xEB
    INSTRUCTION(cpx, ABSOLUTE, 4, false),      // 0xEC
    INSTRUCTION(sbc, ABSOLUTE, 4, false),      // 0xED
    INSTRUCTION(inc, ABSOLUTE, 6, false),      // 0xEE
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xEF
    INSTRUCTION(beq, RELATIVE, 2, true),       // 0xF0
    INSTRUCTION(sbc, POST_INDIRECT, 5, true),  // 0xF1
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xF2
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xF3
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xF4
    INSTRUCTION(sbc, ZERO_X, 4, false),        // 0xF5
    INSTRUCTION(inc, ZERO_X, 6, false),        // 0xF6
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xF7
    INSTRUCTION(sed, IMPLIED, 2, false),       // 0xF8
    INSTRUCTION(sbc, ABS_Y, 4, true),          // 0xF9
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xFA
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xFB
    INSTRUCTION(nop, IMPLIED, 2, false),       // 0xFC
    INSTRUCTION(sbc, ABS_X, 4, true),          // 0xFD
    INSTRUCTION(inc, ABS_X, 7, false),         // 0xFE
};

int execute(cpu_t *cpu) {
    instruction_t i = INSTRUCTION_LOOKUP[READ_8(cpu)];
    i.inst(cpu, get_oper_ptr(cpu, i.mode, i.check_boundary));
    cpu->cycles += i.cycles;

    return 0;
}
