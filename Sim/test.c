/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  C test code                                          *
 *                                                       *
 *********************************************************/

/*your test code here*/

// void test_instructions(void) {
//     // Data processing instructions
//     __asm__ volatile("and r0, r1");         // AND (register)
//     __asm__ volatile("eor r2, r3");         // EOR (register)
//     __asm__ volatile("lsl r4, r5");         // LSL (register)
//     __asm__ volatile("lsr r6, r7");         // LSR (register)
//     __asm__ volatile("asr r0, r1");         // ASR (register)
//     __asm__ volatile("adc r2, r3");         // ADC (register)
//     __asm__ volatile("sbc r4, r5");         // SBC (register)
//     __asm__ volatile("ror r6, r7");         // ROR (register)
//     __asm__ volatile("tst r0, r1");         // TST (register)
//     __asm__ volatile("cmp r4, r5");         // CMP (register)
//     __asm__ volatile("cmn r6, r7");         // CMN (register)
//     __asm__ volatile("orr r0, r1");         // ORR (register)
//     __asm__ volatile("mul r2, r3, r2");     // MUL
//     __asm__ volatile("bic r4, r5");         // BIC (register)
//     __asm__ volatile("mvn r6, r7");         // MVN (register)

//     // Immediate operations
//     __asm__ volatile("mov r0, #0x55");      // MOV (immediate)
//     __asm__ volatile("cmp r1, #0xAA");      // CMP (immediate)
//     __asm__ volatile("add r2, #0x3");       // ADD (8-bit)
//     __asm__ volatile("sub r3, #0x7");       // SUB (8-bit)
//     __asm__ volatile("revsh r4, r3");

//     // Memory operations
//     volatile unsigned int buffer[4];
//     __asm__ volatile("stmia r0!, {r1-r3}"); // STM
//     __asm__ volatile("ldmia r0!, {r4-r6}"); // LDM

//     // Special instructions
//     __asm__ volatile("add r1, sp, #0x10");  // ADD (SP immediate)
// }

// int main(void) {
//     test_instructions();
//     return 0;
// }

#include <stdio.h>

void sort_string(const char *src, char *dst, int len) {
    int i, j;
    for(i = 0; i < len; i++) dst[i] = src[i];
    dst[len] = '\0';
    for(i = 0; i < len-1; i++) {
        for(j = i+1; j < len; j++) {
            if(dst[i] > dst[j]) {
                char tmp = dst[i];
                dst[i] = dst[j];
                dst[j] = tmp;
            }
        }
    }
}

int main() {
    char x[13] = "QWERTYASDFGH"; 
    char y[13];
    sort_string(x, y, 12);
    return 0;
}
