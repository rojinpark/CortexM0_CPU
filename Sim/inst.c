/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  C simulator for Cortex-M0                            *
 *                                                       *
 *********************************************************/

#include "iss.h"

/* Return the instruction for EXE pipeline stage */
uint16_t fetch(void)
{
  return read_halfword(EXE_PC);
}

/* Update PC. If branch has occured, PC + 4 for EXE pipeline stage,
   otherwise increment by 2. */
void updatePC(void)
{
  if (!branch){
    PC = PC + 2;
    EXE_PC = PC-4;
    }
  else{
    branch = 0;
    EXE_PC = PC;
    PC = PC + 4;
  }
}

/* Extract a number from DATA at the bit positions START to END.
   Ex) extract(0x00010f02, 8, 16) = 0x10f = 271 */
uint32_t extract32(uint32_t data, int end, int start)
{
  int i;
  uint32_t flag = 0;

  assert(end >= start && 31 >= end && start >= 0);

  for (i = start; i <= end; i++)
    flag = flag + (1 << i);

  return (data & flag) >> start;
}

/* Single bit version of the function EXTRACT32. */
uint32_t extract32_(uint32_t data, int pos)
{
  return extract32(data, pos, pos);
}

/* EXTRACT for uint16_t */
uint16_t extract16(uint16_t data, int end, int start)
{
  int i;
  uint16_t flag = 0;

  assert(end >= start && 15 >= end && start >= 0);

  for (i = start; i <= end; i++)
    flag = flag + (1 << i);

  return (data & flag) >> start;
}

/* Single bit version of the function EXTRACT16. */
uint16_t extract16_(uint16_t data, int pos)
{
  return extract16(data, pos, pos);
}

/* A is a variable of length LENGTH bits
 * This function sign extends A to 32 bits */
uint32_t sign_extend(uint32_t a, int length)
{
  int i;
  unsigned flag = 0x80000000, result = a;

  if (extract32_(a, length - 1)) {
    for (i = 31; i >= length; i--) {
      result = result + flag;
      flag = flag >> 1;
    }
    return result;
  }
  else
    return a;
}

