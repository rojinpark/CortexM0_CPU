/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  C simulator for Cortex-M0                            *
 *                                                       *
 *********************************************************/


#include "iss.h"

void b_unconditional(uint16_t inst);
void bl(uint32_t inst);

void process(uint16_t inst)
{
  uint16_t inst2;
  uint32_t inst32;

  if (INST(15, 11) == 0x3) {add_sub(inst);}                           // add, sub (reg, 3bit imm)
  else if(INST(15,12) == 0x3) {add_sub_imm8(inst);}                     // add, sub (8bit imm)
  else if(INST(15,12) == 0x0) {ls(inst, 1);}                            // lsl, lsr for imm
  else if(INST(15, 7) == 0x81) {ls(inst, 0);}                           // lsl, lsr for reg
  else if(INST(15,11) == 0x2) {asr(inst, 1);}                           // asr for imm
  else if(INST(15,6) == 0x104) {asr(inst, 0);}                         // asr for reg
  else if(INST(15,11) == 0x4) {mov(inst, 1);}                         // mov for imm
  else if(INST(15,8) == 0x46) {mov(inst, 0);}                           // mov for reg
  else if(INST(15,11) == 0x5) {cmp(inst, 0);}                           // cmp for imm
  else if(INST(15,10) == 0x10) {group1(inst);}                          // AND ~ MVN
  else if(INST(15,10) == 0x11) {group2(inst);}                          // ADD(reg2) ~ BLX
  else if(INST(15,11) == 0x9) {ldr_pc(inst);}                              // LDR(PC-relative)
  else if(INST(15,12) == 0x5) {ldst_reg(inst);}                         // STR ~ LDRSH
  else if(INST(15,13) == 0x3 || INST(15,13) == 0x4) {ldst_imm(inst);}   // STR(5bit imm) ~ LDR(8bit imm)
  else if(INST(15,12) == 0xA) {adr_add(inst);}                          // ADR, ADD(SP imm1)
  else if(INST(15,12) == 0xB && INST(11,8) == 0x0) {add_sub_sp(inst);}  // ADD(SP imm2), SUB(SP imm2)
  else if(INST(15,12) == 0xB && INST(11,8) == 0x2) {us_extract(inst);}  // SXTH, SXTB, UXTH, UXTB
  else if(INST(15,12) == 0xB && INST(11,8) == 0xF) {no_ops(inst);}      // NOP ~ SEV
  else if(INST(15,12) == 0xD) {b_conditional(inst);}                    // conditional B
  else if(INST(15,12) == 0xB && INST(11,9) == 0x2) {push(inst);}        // push
  else if(INST(15,12) == 0xB && INST(11,9) == 0x6) {pop(inst);}         // pop
  else if(INST(15,12) == 0xC) {stldm(inst);}                            // STM, LDM
  else if(INST(15,9) == 0x5D) {rev(inst);}                              // REV, REV16, REVSH
  else if (INST(15, 11) == 0x1C) {
    b_unconditional(inst);
  }
  else if (INST(15, 11) == 0x1E) {
    inst2 = read_halfword(EXE_PC + 2);
    inst32 = ((uint32_t) inst << 16) | ((uint32_t) inst2);
    if (extract16_(inst2, 14) && extract16_(inst2, 12))
      bl(inst32);
  }
}

    /* your code here (for additional functions)*/


/* Conduct add operation. sub operation can be conducted as well. */
/* N, Z, C, V are updated */
uint32_t AddWithCarry(uint32_t x, uint32_t y, int carry_in)
{
  uint64_t sum64 = (uint64_t)x + y + carry_in;
  uint32_t result = (uint32_t)sum64;

  APSR.C = (sum64>>32) ? 1:0; // check carry

  uint32_t x_sign = x >> 31;
  uint32_t y_sign = y >> 31;
  uint32_t sum_sign = result >> 31;

  APSR.V = (x_sign == y_sign) && (sum_sign != x_sign); // check overflow

  APSR.N = (int)MSB(result);
  APSR.Z = (result == 0); 
  return result;
}

/* Conduct lsl, lsr for imm as well as reg */
/* N, Z, C are updated */
void ls(uint16_t inst, int Whether_Imm)
{
  unsigned int index1 = INST_(11); // 0 for lsl, 1 for lsr (for imm)
  unsigned int index2 = INST_(6); // 0 for lsl, 1 for lsr (for reg)
  unsigned int srcm = INST(5,3);
  unsigned int dest = INST(2,0);
  unsigned int imm = INST(10,6);

  if(Whether_Imm) {
    if(!index1) {
      printf("LSL_imm Called\nStore R[%d] << %08x into R[%d]\n\n", srcm, imm, dest);
      R[dest] = R[srcm] << imm;
    } // lsl for imm
    else {
      printf("LSR_imm Called\nStore R[%d] >> %08x into R[%d]\n\n", srcm, imm, dest);
      if(imm == 0) imm = 32;
      R[dest] = R[srcm] >> imm; 
    } // lsr for imm
    if(imm) { // update carry flag
      if(!index1) APSR.C = (int)MSB(R[srcm] << (imm-1));
      else APSR.C = (int)((R[srcm] >> (imm-1))%2);
    }
  }

  else {
    if((R[srcm] & 0xFF)) { // update carry flag
      if(!index2) APSR.C = (int)MSB(R[dest] << ((R[srcm] & 0xFF)-1));
      else APSR.C = (int)( (R[dest] >> ((R[srcm] & 0xFF)-1)) % 2);
    }
    if(!index2) {
      printf("LSL_reg Called\nStore R[%d] << R[%d] into R[%d]\n\n", dest, srcm, dest);
      R[dest] = R[dest] << (R[srcm] & 0xFF); 
    } // lsl for reg
    else {
      printf("LSR_reg Called\nStore R[%d] >> R[%d] into R[%d]\n\n", dest, srcm, dest);
      R[dest] = R[dest] >> (R[srcm] & 0xFF); 
    } // lsr for reg
  }

  APSR.N = (int)MSB(R[dest]);
  APSR.Z = (R[dest] == 0);
}

/* Conduct asr for imm as well as reg */
/* N, Z, C are updated*/
void asr(uint16_t inst, int Whether_Imm)
{
  unsigned int srcm = INST(5,3);
  unsigned int dest = INST(2,0);
  unsigned int imm = INST(10,6);
  uint32_t temp_m = R[srcm];
  uint32_t temp_d = R[dest];
  uint32_t msb_m = (MSB(R[srcm]) << 31); // most significant bit of R[srcm]
  uint32_t msb_d = (MSB(R[dest]) << 31); // most significant bit of R[dest]

  if(Whether_Imm) {
    printf("ASR_imm Called\nStore R[%d] >> %08x into R[%d]\n\n", srcm, imm, dest);
    if(imm == 0) imm = 32;
    for(int i = 0; i < imm; i++){
      if(i == imm -1) {
        APSR.C = (int)(temp_m % 2); // update APSR.C
      }
      temp_m = msb_m + (temp_m >> 1);
    }
    R[dest] = temp_m;
  }

  else {
    printf("ASR_reg Called\nStore R[%d] >> R[%d] into R[%d]\n\n", dest, srcm, dest);
    for(int j = 0; j < (R[srcm] & 0xFF); j++){
      if(j == (R[srcm] & 0xFF) - 1) {
        APSR.C = (int)(temp_d % 2); // update APSR.C
      }
      temp_d = msb_d + (temp_d >> 1);
    }
    R[dest] = temp_d;
  }

  APSR.N = (int)MSB(R[dest]);
  APSR.Z = (R[dest] == 0);
}

/* ADD, SUB for 3bit imm and reg1 */
/* N, Z, C, V are updated */
void add_sub(uint16_t inst)
{
  unsigned int index = INST(10,9);
  uint32_t srcm = INST(8,6);
  unsigned int srcn = INST(5,3);
  unsigned int dest = INST(2,0);

  if(index == 0){ // ADD(reg)
    printf("ADD_reg1 Called\nStore R[%d] + R[%d] into R[%d]\n\n", srcn, srcm, dest);
    R[dest] = AddWithCarry(R[srcn], R[srcm], 0);
  }
  else if(index == 1){ // SUB(reg)
    printf("SUB_reg Called\nStore R[%d] - R[%d] into R[%d]\n\n", srcn, srcm, dest);
    R[dest] = AddWithCarry(R[srcn], ~(R[srcm]), 1);
  }
  else if(index == 2){ // ADD(3bit imm)
    printf("ADD_3bitimm Called\nStore R[%d] + %08x into R[%d]\n\n", srcn, srcm, dest);
    R[dest] = AddWithCarry(R[srcn], srcm, 0);
  }
  else{ // SUB(3bit imm)
    printf("SUB_3bitimm Called\nStore R[%d] - %08x into R[%d]\n\n", srcn, srcm, dest);
    R[dest] = AddWithCarry(R[srcn], ~srcm, 1);
  }
}

/* ADD, SUB for 8bit imm */
/* N, Z, C, V are updated */
void add_sub_imm8(uint16_t inst) 
{
  unsigned int index = INST_(11);
  unsigned int dest = INST(10,8);
  uint32_t imm = INST(7,0);

  if(index == 0){
    printf("ADD_imm8 Called\nStore R[%d] + %08x into R[%d]\n\n", dest, imm, dest);
    R[dest] = AddWithCarry(R[dest], imm, 0);
  }
  else{
    printf("SUB_imm8 Called\nStore R[%d] - %08x into R[%d]\n\n", dest, imm, dest);
    R[dest] = AddWithCarry(R[dest], ~imm, 1);
  }
}

/* ADD(SP imm2), SUB(SP imm) */
/* No APSR updates */
void add_sub_sp(uint16_t inst)
{
  uint32_t imm = INST(6,0);

  if(!INST_(7)){ // ADD(SP imm2)
    printf("ADD_SPimm2 Called\nStore SP + %08x * 4 into SP\n\n", imm);
    SP = SP + imm*4;
  }
  else { // SUB(SP imm)
    printf("SUB_SPimm Called\nStore SP - %08x * 4 into SP\n\n", imm);
    SP = SP - imm*4;
  }
}

/* MOV(imm), MOV(reg1) */
/* N, Z are updated only for MOV(imm). No APSR updates for MOV(reg1) */
void mov(uint16_t inst, int Whether_Imm) 
{
  unsigned int dest1 = INST(10,8); // for imm
  unsigned int dest2 = INST_(7) * 8 + INST(2,0); // for reg1
  unsigned int srcm = INST(6,3); // for reg1
  uint32_t imm = INST(7,0); // for imm

  if(Whether_Imm) {
    printf("MOV_imm Called\nStore %08x into R[%d]\n\n", imm, dest1);
    R[dest1] = imm;
    APSR.N = (int)MSB(R[dest1]);
    APSR.Z = (R[dest1] == 0);
  }
  else {
    printf("MOV_reg Called\nStore R[%d] into R[%d]\n\n", srcm, dest2);
    R[dest2] = R[srcm];
  }
}

/* mode0 : CMP(imm) / mode1 : CMP(reg1) / mode2 : CMP(reg2) */
/* N, Z, C, V are updated */
void cmp(uint16_t inst, int mode) 
{
  unsigned int srcn1 = INST(10,8);
  unsigned int srcn2 = INST(2,0);
  unsigned int srcn3 = INST(2,0) + INST_(7)*8; // generate proper regsiter addr
  unsigned int srcm1 = INST(5,3);
  unsigned int srcm2 = INST(6,3);
  uint32_t imm = INST(7,0);

  if(mode == 0) {
    printf("CMP(imm) Called\nCompare R[%d] and %08x\n\n", srcn1, imm);
    AddWithCarry(R[srcn1], ~imm, 1);
  }
  else if(mode == 1) {
    printf("CMP(reg1) Called\nCompare R[%d] and R[%d]\n\n", srcn2, srcm1);
    AddWithCarry(R[srcn2], ~(R[srcm1]), 1);
  }
  else if(mode == 2) {
    printf("CMP(reg1) Called\nCompare R[%d] and R[%d]\n\n", srcn3, srcm2);
    AddWithCarry(R[srcn3], ~(R[srcm2]), 1);
  }
}

/* LDR(PC-relative) */
/* No APSR updates */
void ldr_pc(uint16_t inst)
{
  unsigned int srct = INST(10,8);
  uint32_t imm = INST(7,0)*4;
  printf("LDR_PCrelative Called\nLoad mem(%08x) to R[%d]\n\n", imm + (PC & 0xFFFFFFFC), srct);
  R[srct] = read_word(imm + (PC & 0xFFFFFFFC)); // Word-aligned PC + imm is the addr
}

/* ADR, ADD(SP imm1) */
/* No APSR updates */
void adr_add(uint16_t inst)
{
  unsigned int dest = INST(10,8);
  uint32_t imm = INST(7,0);

  if(!INST_(11)){ // ADR
    printf("ADR Called\nStore align(PC) + %08x * 4 to R[%d]\n\n", imm, dest);
    R[dest] = (PC & 0xFFFFFFFC) + imm*4;
  }
  else{ // ADD(SP imm1)
    printf("ADD_SPimm1 Called\nStore SP + %08x * 4 to R[%d]\n\n", imm, dest);
    R[dest] = SP + imm*4;
  }
}

/* SXTH, SXTB, UXTH, UXTB */
/* No APSR updates */
void us_extract(uint16_t inst)
{
  unsigned int dest = INST(2,0);
  unsigned int srcm = INST(5,3);
  unsigned int index = INST(7,6);

  switch(index){
    case 0 : { // SXTH. Signed extend halfword 
      printf("SXTH Called\nStore halfword sign extended R[%d] to R[%d]\n\n", srcm, dest);
      R[dest] = signExtend32((R[srcm] & 0xFFFF), 16);
      break;
    }
    case 1 : { // SXTB. 
      printf("SXTB Called\nStore byte sign extended R[%d] to R[%d]\n\n", srcm, dest);
      R[dest] = signExtend32((R[srcm] & 0xFF), 8);
      break;
    }
    case 2 : { // UXTH. Unsigned extend halfword
      printf("UXTH Called\nStore halfword unsign extended R[%d] to R[%d]\n\n", srcm, dest);
      R[dest] = zeroExtend32(R[srcm] & 0xFFFF);
      break;
    }
    case 3 : { // UXTB
      printf("UXTB Called\nStore byte unsign extended R[%d] to R[%d]\n\n", srcm, dest);
      R[dest] = zeroExtend32(R[srcm] & 0xFF);
      break;
    }
  }
}

/* NOP ~ SEV. I will not implement this part except for NOP 
since these are associated with interrupts. */
void no_ops(uint16_t inst)
{
  unsigned int index = INST(7,4);

  switch(index){
    case 0 : { // NOP
      printf("NOP Called\nDo nothing\n\n");
      break;
    }
    case 1 : { // YIELD
      break;
    }
    case 2 : { // WFE
      break;
    }
    case 3 : { // WFI
      break;
    }
    case 4 : { // SEV
      break;
    }
  }
}

/* push */
/* No APSR updates */
void push(uint16_t inst)
{
  int Reg[9]; // Check valid regs. Note that Reg[8] expresses LR's validity
  int regcount = 0; // How many valid regs are in reg_list of the instruction
  uint32_t reglist = INST(8,0);
  uint32_t addr;

  for(int i = 0; i < 9; i++){
    Reg[i] = (reglist >> i) % 2; // 0, 1 are stored in all indexes
    if(Reg[i]) {regcount++;}
  }

  printf("push Called\nPush reglist %9b (msb represents LR) into the stack\n\n", reglist);

  addr = SP - 4*regcount;

  for(int j = 0; j < 9; j++){
    if(Reg[j] && j<8) { // jth register is valid and it's not LR
      write_word(addr, R[j]);
      addr = addr + 4;
    }
    else if(Reg[j] && j==8) { // LR is valid
      write_word(addr, LR);
      addr = addr + 4;
    }
  }

  SP = SP - 4*regcount;
}

/* pop */
/* No APSR updates */
void pop(uint16_t inst)
{
  int Reg[9]; // Check valid regs. Note that Reg[8] expresses PC's validity
  int regcount = 0; // How many valid regs are in reg_list of the instruction
  uint32_t reglist = INST(8,0);
  uint32_t addr = SP;

  printf("pop Called\nPop reglist %9b (msb represents PC) from the stack\n\n", reglist);

  for(int i = 0; i < 9; i++){
    Reg[i] = (reglist >> i) % 2; // 0, 1 are stored in all indexes
    if(Reg[i]) {regcount++;}
  }

  for(int j = 0; j < 9; j++){
    if(Reg[j] && j<8) { // jth reg is valid and it's not PC
      R[j] = read_word(addr);
      addr = addr + 4;
    }
    else if(Reg[j] && j==8) { // PC is valid. Branch.
      branch = 1;
      PC = read_word(addr) & 0xFFFFFFFE;
      addr = addr + 4;
    }
  }

  SP = SP + 4*regcount;
}

/* STM, LDM */
/* Similarly operates as push and pop */
/* No APSR updates */
void stldm(uint16_t inst)
{
  int Reg[8];
  int regcount = 0;
  unsigned int srcn = INST(10,8);
  int index = INST_(11);
  uint32_t reglist = INST(7,0);
  uint32_t addr = R[srcn];

  for(int i = 0; i < 8; i++){
    Reg[i] = (reglist >> i) % 2;
    if(Reg[i]) {regcount++;}
  }

  if(!index) { // STM
    printf("STM Called\nStore reglist %8b into the mem(%08x)(starting point)\n\n", reglist, addr);
    for(int i = 0; i < 8; i++){
      if(Reg[i]){
        write_word(addr, R[i]);
        addr = addr + 4;
      }
    }
    R[srcn] = R[srcn] + 4*regcount;
  }
  else { // LDM
    printf("LDM Called\nStore reglist %8b into the mem(%08x)(starting point)\n\n", reglist, addr);
    for(int i = 0; i<8; i++){
      if(Reg[i]){
        R[i] = read_word(addr);
        addr = addr + 4;
      }
    }
    if(!Reg[srcn]){
      R[srcn] = R[srcn] + 4*regcount;
    }
  }
}

/* REV, REV16, REVSH */
/* No APSR updates */
void rev(uint16_t inst)
{
  unsigned int index = INST(7,6);
  unsigned int srcm = INST(5,3);
  unsigned int dest = INST(2,0);

  switch(index){
    case 0 : { // REV
      printf("REV Called\nStore reversed R[%d] into R[%d] \n\n", srcm, dest);
      R[dest] = ((R[srcm] & 0xFF) << 24) + ((R[srcm] & 0xFF00) << 8) + ((R[srcm] & 0xFF0000) >> 8) + ((R[srcm] & 0xFF000000) >> 24);
      break;
    }
    case 1 : { // REV16
      printf("REV16 Called\nStore 16 reversed R[%d] into R[%d] \n\n", srcm, dest);
      R[dest] = ((R[srcm] & 0xFF0000) << 8) + ((R[srcm] & 0xFF000000) >> 8) + ((R[srcm] & 0xFF) << 8) + ((R[srcm] & 0xFF00) >> 8);
      break;
    }
    case 3 : { // REVSH
      printf("REVSH Called\nStore half reversed R[%d] into R[%d] \n\n", srcm, dest);
      R[dest] = (sign_extend((R[srcm] & 0xFF), 8) << 8) + ((R[srcm] & 0xFF00) >> 8);
      break;
    }
  }
}

/* AND ~ MVN. for INST(15,10) == 010000 */
void group1(uint16_t inst)
{
  unsigned int reg1 = INST(5,3); // usually src reg
  unsigned int reg2 = INST(2,0); // usually dest reg

  switch(INST(9,6)) {
    case 0 : { // AND(reg). Bitwise AND. N, Z are updated
      printf("AND_reg Called\nStore R[%d] & R[%d] into R[%d] \n\n", reg2, reg1, reg2);
      R[reg2] = R[reg2] & R[reg1]; 
      APSR.N = (int)MSB(R[reg2]);
      APSR.Z = (R[reg2] == 0);
      break;
    }
    case 1 : { // EOR(reg). Bitwise XOR. N, Z are updated
      printf("EOR_reg Called\nStore R[%d] ^ R[%d] into R[%d] \n\n", reg2, reg1, reg2);
      R[reg2] = R[reg2] ^ R[reg1]; 
      APSR.N = (int)MSB(R[reg2]);
      APSR.Z = (R[reg2] == 0);
      break;
    }
    case 5 : { // ADC(reg). Add with carry. Note that 2,3,4 are already processed by ls and asr functions
      printf("ADC_reg Called\nStore R[%d] + R[%d] + APSR.C into R[%d] \n\n", reg2, reg1, reg2);
      R[reg2] = AddWithCarry(R[reg1], R[reg2], APSR.C); // N, Z, C, V are updated
      break;
    }
    case 6 : { // SBC(reg). Substract with carry
      printf("SBC_reg Called\nStore R[%d] - R[%d] + APSR.C into R[%d] \n\n", reg2, reg1, reg2);
      R[reg2] = AddWithCarry(R[reg2], ~(R[reg1]), APSR.C); // N, Z, C, V are updated
      break;
    }
    case 7 : { // ROR(reg). Rotate right. N, Z, C are updated
      printf("ROR_reg Called\nRotate R[%d] right by R[%d] and store into R[%d] \n\n", reg2, reg1, reg2);
      int count = 0;
      uint32_t temp = R[reg2];
      uint32_t lsb = 0;
      for(count = 0; count < (R[reg1] & 0xFF); count++){
        if(count == ((R[reg1] & 0xFF) - 1)) {
          APSR.C = (int)(temp % 2);
        }
        lsb = (temp % 2);
        temp = ((lsb << 31) + (temp >> 1));
      }
      R[reg2] = temp;
      APSR.N = (int)MSB(temp);
      APSR.Z = (temp == 0);
      break;
    }
    case 8 : { // TST(reg). State register update with AND operation. N, Z are updated
      printf("TST_reg Called\nUpdate APSR after R[%d] & R[%d] \n\n", reg1, reg2);
      uint32_t temp8;
      temp8 = R[reg1] & R[reg2];
      APSR.N = (int)MSB(temp8);
      APSR.Z = (temp8 == 0);
      break;
    }
    case 9 : { // RSB(imm). Substract from 0. N, Z, C, V are updated
      printf("RSB_reg Called\nSubstract R[%d] from 0 and store into R[%d] \n\n", reg1, reg2);
      R[reg2] = AddWithCarry(~(R[reg1]), 0, 1);
      break;
    }
    case 10 : { // CMP(reg1). N, Z, C, V are updated
      cmp(inst, 1);
      break;
    }
    case 11 : { // CMN(reg). Compare negative. Note that 10 is already processed by cmp function
      printf("CMN_reg Called\nUpdate APSR after R[%d] + R[%d] \n\n", reg1, reg2);
      AddWithCarry(R[reg1], R[reg2], 0); // N, Z, C, V are updated
      break;
    }
    case 12 : { // ORR(reg). Bitwise OR. N, Z are updated
      printf("ORR_reg Called\nStore R[%d] | R[%d] into R[%d] \n\n", reg2, reg1, reg2);
      R[reg2] = R[reg2] | R[reg1]; 
      APSR.N = (int)MSB(R[reg2]);
      APSR.Z = (R[reg2] == 0);
      break;
    }
    case 13 : { // MUL(reg). Multiply. N, Z are updated
      printf("MUL_reg Called\nStore R[%d] * R[%d] into R[%d] \n\n", reg2, reg1, reg2);
      R[reg2] = R[reg2] * R[reg1];
      APSR.N = (int)MSB(R[reg2]);
      APSR.Z = (R[reg2] == 0); 
      break;
    }
    case 14 : { // BIC(reg). Bitwise Bit Clear. N, Z are updated
      printf("BIC_reg Called\nStore R[%d] & (~R[%d]) into R[%d] \n\n", reg2, reg1, reg2);
      R[reg2] = R[reg2] & (~R[reg1]);
      APSR.N = (int)MSB(R[reg2]);
      APSR.Z = (R[reg2] == 0);
      break;
    }
    case 15 : { // MVN(reg). Bitwise NOT. N, Z are updated
      printf("NOT_reg Called\nStore ~R[%d] into R[%d] \n\n", reg1, reg2);
      R[reg2] = (~R[reg1]);
      APSR.N = (int)MSB(R[reg2]);
      APSR.Z = (R[reg2] == 0);
      break;
    }
    default : break;
  }
}

/* ADD(reg2) ~ BLX. for INST(15,10) == 010001 */
void group2(uint16_t inst)
{
  unsigned int dest = INST(2,0) + 8*INST_(7);
  unsigned int srcm = INST(6,3);
  unsigned int index = INST_(7);

  switch(INST(9,8)){
    case 0 : { // ADD(reg2). No APSR updates
      printf("ADD_reg2 Called\nStore R[%d] + R[%d] into R[%d] \n\n", dest, srcm, dest);
      R[dest] = R[dest] + R[srcm];
      break;
    }
    case 1 : { // CMP(reg2). N, Z, C, V are updated
      cmp(inst, 2);
      break;
    }
    case 3 : { // BX and BLX. Note that case 2, which is MOV(reg1), is already processed by mov function.
      if(!index){ // BX. No APSR updates
        printf("BX Called\nNextPC = R[%d] \n\n", srcm);
        branch = 1;
        PC = R[srcm] & 0xFFFFFFFE;
      }
      else{ // BLX. No APSR updates
        printf("BLX Called\nLR = (PC - 2) | 0x1, NextPC = R[%d] \n\n", srcm);
        LR = (PC-2) | 0x1;
        branch = 1;
        PC = R[srcm] & 0xFFFFFFFE;
      }
      break;
    }
  }
}

/* STR ~ LDRSH. for INST(15,12) == 0101 */
/* No APSR updates */
void ldst_reg(uint16_t inst)
{
  unsigned int srcm = INST(8,6);
  unsigned int srcn = INST(5,3);
  unsigned int srct = INST(2,0);
  uint32_t temp = 0;
  int msb = 0;

  switch(INST(11,9)){
    case 0 : { // STR(reg)
      printf("STR_reg Called\nStore R[%d] into mem(R[%d]+R[%d]) \n\n", srct, srcn, srcm);
      write_word(R[srcn]+R[srcm], R[srct]);
      break;
    }
    case 1 : { // STRH(reg)
      printf("STRH_reg Called\nStore halfword of R[%d] into mem(R[%d]+R[%d]) \n\n", srct, srcn, srcm);
      write_halfword(R[srcn]+R[srcm], R[srct] & 0xFFFF);
      break;
    }
    case 2 : { // STRB(reg)
      printf("STRB_reg Called\nStore byte of R[%d] into mem(R[%d]+R[%d]) \n\n", srct, srcn, srcm);
      write_byte(R[srcn]+R[srcm], R[srct] & 0xFF);
      break;
    }
    case 3 : { // LDRSB(reg) load reg signed byte
      printf("LDRSB_reg Called\nLoad R[%d] from signed byte of mem(R[%d]+R[%d]) \n\n", srct, srcn, srcm);
      temp = read_byte(R[srcn]+R[srcm]);
      if(MSB(temp<<24)) {
        R[srct] = 0xFFFFFF00 + temp;
      }
      else {
        R[srct] = temp;
      }
      break;
    }
    case 4 : { // LDR(reg)
      printf("LDR_reg Called\nLoad R[%d] from mem(R[%d]+R[%d]) \n\n", srct, srcn, srcm);
      R[srct] = read_word(R[srcn]+R[srcm]);
      break;
    }
    case 5 : { // LDRH(reg)
      printf("LDRH_reg Called\nLoad R[%d] from halfword of mem(R[%d]+R[%d]) \n\n", srct, srcn, srcm);
      R[srct] = read_halfword(R[srcn]+R[srcm]);
      break;
    }
    case 6 : { // LDRB(reg)
      printf("LDRB_reg Called\nLoad R[%d] from halfword of mem(R[%d]+R[%d]) \n\n", srct, srcn, srcm);
      R[srct] = read_byte(R[srcn]+R[srcm]);
      break;
    }
    case 7 : { // LDRSH(reg) load reg signed halfword
      printf("LDRSH_reg Called\nLoad R[%d] from signed halfword of mem(R[%d]+R[%d]) \n\n", srct, srcn, srcm);
      temp = read_halfword(R[srcn]+R[srcm]);
      if(MSB(temp<<16)) {
        R[srct] = 0xFFFF0000 + temp;
      }
      else {
        R[srct] = temp;
      }
      break;
    }
  }
}

/* STR(5bit imm) ~ LDR(8bit imm) for INST(15,13) == 011 or 100 */
/* No APSR updates */
void ldst_imm(uint16_t inst)
{
  unsigned int srcn = INST(5,3);
  unsigned int srct = INST(2,0);
  unsigned int srct_8bitimm = INST(10,8);
  uint32_t imm5 = INST(10,6);
  uint32_t imm8 = INST(7,0);

  if(INST(15,13) == 0x3){
    switch(INST(12,11)){
      case 0 : { // STR(5bit imm)
        printf("STR_5bitimm Called\nStore R[%d] into mem(R[%d] + 4 * %08x) \n\n", srct, srcn, imm5);
        write_word(R[srcn] + imm5*4, R[srct]);
        break;
      }
      case 1 : { // LDR(5bit imm)
        printf("LDR_5bitimm Called\nLoad R[%d] from mem(R[%d] + 4 * %08x) \n\n", srct, srcn, imm5);
        R[srct] = read_word(R[srcn] + imm5*4);
        break;
      }
      case 2 : { // STRB(imm)
        printf("STRB_imm Called\nStore byte of R[%d] into mem(R[%d] + 4 * %08x) \n\n", srct, srcn, imm5);
        write_byte(R[srcn] + imm5, R[srct]);
        break;
      }
      case 3 : { // LDRB(imm)
        printf("LDRB_imm Called\nLoad R[%d] from byte of mem(R[%d] + 4 * %08x) \n\n", srct, srcn, imm5);
        R[srct] = read_byte(R[srcn] + imm5);
        break;
      }
    }
  }

  else if(INST(15,13) == 0x4){
    switch(INST(12,11)){
      case 0 : { // STRH(imm)
        printf("STRH_imm Called\nStore halfword of R[%d] into mem(R[%d] + 4 * %08x) \n\n", srct, srcn, imm5);
        write_halfword(R[srcn] + imm5*2, R[srct]);
        break;
      }
      case 1 : { // LDRH(imm)
        printf("LDRH_imm Called\nLoad R[%d] from halfword of mem(R[%d] + 4 * %08x) \n\n", srct, srcn, imm5);
        R[srct] = read_halfword(R[srcn] + imm5*2);
        break;
      }
      case 2 : { // STR(8bit imm)
        printf("STR_8bitimm Called\nStore R[%d] into mem(SP + 4 * %08x) \n\n", srct, imm8);
        write_word(SP + imm8*4, R[srct]);
        break;
      }
      case 3 : { // LDR(8bit imm)
        printf("LDR_8bitimm Called\nLoad R[%d] from mem(SP + 4 * %08x) \n\n", srct, imm8);
        R[srct] = read_word(SP + imm8*4);
        break;
      }
    }
  }
}

/* Conditional branch instruction */
/* No APSR updates */
void b_conditional(uint16_t inst)
{
  unsigned int cond = INST(11,8);
  uint32_t imm = INST(7,0);
  uint32_t addr;
  int Whether_Branch = 0;

  switch(cond){
    case 0 : { // EQ
      printf("B_conditional Called - EQ \n");
      if(APSR.Z == 1) Whether_Branch = 1;
      break;
    }
    case 1 : { // NE
      printf("B_conditional Called - NE \n");
      if(APSR.Z == 0) Whether_Branch = 1;
      break;
    }
    case 2 : { // CS
      printf("B_conditional Called - CS \n");
      if(APSR.C == 1) Whether_Branch = 1;
      break;
    }
    case 3 : { // CC
      printf("B_conditional Called - CC \n");
      if(APSR.C == 0) Whether_Branch = 1;
      break;
    }
    case 4 : { // MI
      printf("B_conditional Called - MI \n");
      if(APSR.N == 1) Whether_Branch = 1;
      break;
    }
    case 5 : { // PL
      printf("B_conditional Called - PL \n");
      if(APSR.N == 0) Whether_Branch = 1;
      break;
    }
    case 6 : { // VS
      printf("B_conditional Called - VS \n");
      if(APSR.V == 1) Whether_Branch = 1;
      break;
    }
    case 7 : { // VC
      printf("B_conditional Called - VC \n");
      if(APSR.V == 0) Whether_Branch = 1;
      break;
    }
    case 8 : { // HI
      printf("B_conditional Called - HI \n");
      if(APSR.C == 1 && APSR.Z == 0) Whether_Branch = 1;
      break;
    }
    case 9 : { // LS
      printf("B_conditional Called - LS \n");
      if(APSR.C == 0 || APSR.Z == 1) Whether_Branch = 1;
      break;
    }
    case 10 : { // GE
      printf("B_conditional Called - GE \n");
      if(APSR.N == APSR.V) Whether_Branch = 1;
      break;
    }
    case 11 : { // LT
      printf("B_conditional Called - LT \n");
      if(APSR.N != APSR.V) Whether_Branch = 1;
      break;
    }
    case 12 : { // GT
      printf("B_conditional Called - GT \n");
      if(APSR.Z == 0 && APSR.Z == APSR.N) Whether_Branch = 1;
      break;
    }
    case 13 : { // LE
      printf("B_conditional Called - LE \n");
      if(APSR.Z == 1 || APSR.Z != APSR.N) Whether_Branch = 1;
      break;
    }
    case 14 : { // None(AL)
      printf("B_conditional Called - None(AL) \n");
      Whether_Branch = 1;
      break;
    }
  }

  if(Whether_Branch) {
    printf("Branch Resolved \n\n");
    addr = PC + signExtend32((imm << 1), 9);
    branch = 1;
    PC = addr & 0xFFFFFFFE;
  }
  else {
    printf("Branch Not Resolved \n\n");
  }
}

void b_unconditional(uint16_t inst)
{
  uint32_t imm11 = INST(10, 0);
  uint32_t address;

  address = PC + signExtend32((imm11 << 1), 12);
  branch = 1;
  PC = address & 0xFFFFFFFE;
}
void bl(uint32_t inst)
{
  uint32_t S = INST32_(10 + 16);
  uint32_t imm10 = INST32(9 + 16, 0 + 16);
  uint32_t J1 = INST32_(13);
  uint32_t J2 = INST32_(11);
  uint32_t imm11 = INST32(10, 0);
  uint32_t I1, I2, imm32, address;
  
  I1 = !(J1 ^ S);
  I2 = !(J2 ^ S);
  imm32 = sign_extend((S << 24) | (I1 << 23) | (I2 << 22) | (imm10 << 12) | (imm11 << 1), 25);

  LR = PC | 0x00000001;

  address = PC + imm32;
  branch = 1;
  PC = address & 0xFFFFFFFE;
}

