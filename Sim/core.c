/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  C simulator for Cortex-M0                            *
 *                                                       *
 *********************************************************/

#include "iss.h"

void init_mem(char *mem_name);
void init_register(void);
void view_reg(uint16_t inst);

int main(int argc, char *argv[])
{
  uint16_t inst;
  uint32_t bp,mp;
  int n = 0, m = 0;
  int i;
  char in;

  /* Initialization */
  init_mem(argv[1]);
  init_register();

  /* Running a program */
  while (1) {
    printf("Request: ");
    scanf(" %c", &in);

    switch (in) {
    case 's': // process a single instruction
      inst = fetch();
      process(inst);
      view_reg(inst);
      updatePC();
      break;
    case 'm': // process mutiple instructions
      printf("Number of instructions: ");
      scanf(" %d", &m);
      for (i = 0; i < m - 1; i++) {
        inst = fetch();
        process(inst);
        updatePC();
      }
      inst = fetch();
      process(inst);
      view_reg(inst);
      updatePC();
      break;
    case 'b': // process until the breakpoint is reached
      printf("Breakpoint address: ");
      scanf(" %x", &bp);
      while (PC != (uint16_t) bp) {
        inst = fetch();
        process(inst);
        updatePC();
      }
      inst = fetch();
      process(inst);
      view_reg(inst);
      updatePC();
      break;
    case 'v': //memory view
      printf("Memory address: ");
      scanf(" %x",&mp);
      printf("%08x : %08x\n", mp,read_word(mp));
      break;
    case 'q': // quit
      exit(0);
      break;
    default:
      printf("Wrong input!\n");
      while (getchar() != '\n');
      break;
    }
  }
  return 0;
}

void init_mem(char *mem_name)
{
  FILE *fp;
  int i;
	uint32_t temp;

  mem = (unsigned char *) calloc(MEM_SIZE, sizeof(unsigned char));
  assert(mem != NULL);

  fp = fopen(mem_name, "r");
  for (i = 0; i < MEM_SIZE; i = i + 4) {
    if (fscanf(fp, "%x", &temp) <= 0)
      break;
		mem[i] = (unsigned char) temp;
		mem[i + 1] = (unsigned char) (temp >> 8);
		mem[i + 2] = (unsigned char) (temp >> 16);
		mem[i + 3] = (unsigned char) (temp >> 24);
	}
  fclose(fp);
}

void init_register(void)
{
  int i;
  for (i = 0; i < 16; i++)
    R[i] = 0;

  R[13] = read_word(0);
  //In real pipeline ARM core, PC is initialiezed by read_word(4) & 0xFFFFFFFE = 0xD0 which is address of Reset_Handler.
  //However, we add 4 to start simulatoin from EXE pipeline stage.  
  R[15] = (read_word(4) & 0xFFFFFFFE) + 4;
  EXE_PC = PC-4;

  APSR.N = 0;
  APSR.Z = 0;
  APSR.C = 0;
  APSR.V = 0;

  branch = 0;

}

void view_reg(uint16_t inst)
{
  int i;

  printf("\nInst: %04X\n", inst);
  printf("Current PC: %08X\tExcuted PC: %08X\n", PC,EXE_PC);
  printf("Registers\n");
  for (i = 0; i < 16; i++) {
    printf("[%2d]%08X ", i, R[i]);
    if (i % 4 == 3)
      printf("\n");
  }

  printf("NZCV: %d%d%d%d\n", APSR.N, APSR.Z, APSR.C, APSR.V);
  printf("\n===============================================\n");
  printf("\n");
}
