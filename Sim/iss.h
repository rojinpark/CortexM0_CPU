/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  C simulator for Cortex-M0                            *
 *                                                       *
 *********************************************************/

#ifndef ISS_H
#define ISS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>


// Registers
extern uint32_t R[16];

typedef struct APSR {
  int N;
  int Z;
  int C;
  int V;
} APSR_t;

extern APSR_t APSR;

#define SP R[13]
#define LR R[14]
#define PC R[15]

#define MEM_SIZE  0x4000000
extern uint8_t *mem;


// BRANCH should be set to 1 whenever branch occurs
extern int branch;
//instruction address for EXE pipeline stage(PC-4)
extern uint32_t EXE_PC;

uint16_t fetch(void);
void process(uint16_t inst);
void updatePC(void);

uint32_t extract32(uint32_t data, int end, int start);
uint32_t extract32_(uint32_t data, int pos);
uint16_t extract16(uint16_t data, int end, int start);
uint16_t extract16_(uint16_t data, int pos);
uint32_t sign_extend(uint32_t a, int length);

/* Shorthand for extracting bits in inst. */
#define INST(x, y) extract16(inst, x, y)
#define INST_(x) extract16_(inst, x)

#define INST32(x, y) extract32(inst, x, y)
#define INST32_(x) extract32_(inst, x)

#define MSB(x) extract32_(x, 31)

// Sign extension
#define zeroExtend32(x) (x)
#define signExtend32(x, n) (((((x) >> ((n)-1)) & 0x1) != 0) ? (~((unsigned int)0) << (n)) | (x) : (x))

/* Memory access functions. */
uint32_t read_word(uint32_t addr);
uint32_t read_halfword(uint32_t addr);
uint32_t read_byte(uint32_t addr);

void write_word(uint32_t addr, uint32_t value);
void write_halfword(uint32_t addr, uint32_t value);
void write_byte(uint32_t addr, uint32_t value);

/* your code here */
uint32_t AddWithCarry(uint32_t x, uint32_t y, int carry_in);
void ls(uint16_t inst, int Whether_Imm);
void asr(uint16_t inst, int Whether_Imm);
void add_sub(uint16_t inst);
void add_sub_imm8(uint16_t inst);
void add_sub_sp(uint16_t inst);
void mov(uint16_t inst, int Whether_Imm) ;
void cmp(uint16_t inst, int mode);
void ldr_pc(uint16_t inst);
void adr_add(uint16_t inst);
void us_extract(uint16_t inst);
void no_ops(uint16_t inst);
void push(uint16_t inst);
void pop(uint16_t inst);
void stldm(uint16_t inst);
void rev(uint16_t inst);
void group1(uint16_t inst);
void group2(uint16_t inst);
void ldst_reg(uint16_t inst);
void ldst_imm(uint16_t inst);
void b_conditional(uint16_t inst);

#endif
