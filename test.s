/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  ASM Test Code for Cortex-M0                          *
 *                                                       *
 *********************************************************/

  .syntax unified
  .arch armv6-m

/* stack configuration */
  .section .stack
  .align 3
  .equ      Stack_Size, 0x200
  .globl    __StackTop
  .globl    __StackLimit

__StackLimit:
  .space    Stack_Size
  .size     __StackLimit, . - __StackLimit
__StackTop:
  .size     __StackTop, . - __StackTop

/* heap configuration */
  .section .heap
  .align 3
  .equ      Heap_Size, 0x1000
  .globl    __HeapBase
  .globl    __HeapLimit

__HeapBase:
  .space    Heap_Size
  .size     __HeapBase, . - __HeapBase
__HeapLimit:
  .size     __HeapLimit, . - __HeapLimit


/* Vector Table */

  .section .isr_vector
  .align 2
  .globl  __isr_vector
__isr_vector:
  .long   __StackTop                  /* Top of Stack                  */
  .long   Reset_Handler               /* Reset Handler                 */
  .long   NMI_Handler                 /* NMI Handler                   */
  .long   HardFault_Handler           /* Hard Fault Handler            */
  .long   0                           /* Reserved                      */
  .long   0                           /* Reserved                      */
  .long   0                           /* Reserved                      */
  .long   0                           /* Reserved                      */
  .long   0                           /* Reserved                      */
  .long   0                           /* Reserved                      */
  .long   0                           /* Reserved                      */
  .long   SVC_Handler                 /* SVCall Handler                */
  .long   0                           /* Debug Monitor Handler         */
  .long   0                           /* Reserved                      */
  .long   PendSV_Handler              /* PendSV Handler                */
  .long   SysTick_Handler             /* SysTick Handler               */

/* External Interrupts */
  .long   IRQ_Handler0
  .long   IRQ_Handler1
  .long   IRQ_Handler2
  .long   IRQ_Handler3
  .long   IRQ_Handler4
  .long   IRQ_Handler5
  .long   IRQ_Handler6
  .long   IRQ_Handler7
  .long   IRQ_Handler8
  .long   IRQ_Handler9
  .long   IRQ_Handler10
  .long   IRQ_Handler11
  .long   IRQ_Handler12
  .long   IRQ_Handler13
  .long   IRQ_Handler14
  .long   IRQ_Handler15
  .long   IRQ_Handler16
  .long   IRQ_Handler17
  .long   IRQ_Handler18
  .long   IRQ_Handler19
  .long   IRQ_Handler20
  .long   IRQ_Handler21
  .long   IRQ_Handler22
  .long   IRQ_Handler23
  .long   IRQ_Handler24
  .long   IRQ_Handler25
  .long   IRQ_Handler26
  .long   IRQ_Handler27
  .long   IRQ_Handler28
  .long   IRQ_Handler29
  .long   IRQ_Handler30
  .long   IRQ_Handler31
  .long   __StackLimit
  .long   __StackTop
  .long   __HeapBase
  .long   __HeapLimit

  .size    __isr_vector, . - __isr_vector

/* Reset Handler */
  .section .startup
  .thumb
  .thumb_func
  .align 2
  .globl  Reset_Handler
  .type   Reset_Handler, %function
Reset_Handler:
  movs r0, #0
  movs r1, #0
  movs r2, #0
  movs r3, #0
  movs r4, #0
  movs r5, #0
  movs r6, #0
  movs r7, #0
  mov r8, r7
  mov r9, r7
  mov r10, r7
  mov r11, r7
  mov r12, r7
  mov r14, r7
     
  b    asm

  .pool
  .size Reset_Handler, . - Reset_Handler

/*your test code here*/
asm:
  movs r1, #1
  adds r0, r0, r1
  bl   main
  b    asm

  .macro    def_default_handler    handler_name
  .align 1
  .thumb_func
  .weak    \handler_name
  .type    \handler_name, %function
\handler_name :
  b    .
  .size    \handler_name, . - \handler_name
  .endm

/* System Exception Handlers */

  def_default_handler   NMI_Handler
  def_default_handler   HardFault_Handler
  def_default_handler   SVC_Handler
  def_default_handler   PendSV_Handler
  def_default_handler   SysTick_Handler

/* IRQ Handlers */

  def_default_handler   IRQ_Handler0
  def_default_handler   IRQ_Handler1
  def_default_handler   IRQ_Handler2
  def_default_handler   IRQ_Handler3
  def_default_handler   IRQ_Handler4
  def_default_handler   IRQ_Handler5
  def_default_handler   IRQ_Handler6
  def_default_handler   IRQ_Handler7
  def_default_handler   IRQ_Handler8
  def_default_handler   IRQ_Handler9
  def_default_handler   IRQ_Handler10
  def_default_handler   IRQ_Handler11
  def_default_handler   IRQ_Handler12
  def_default_handler   IRQ_Handler13
  def_default_handler   IRQ_Handler14
  def_default_handler   IRQ_Handler15
  def_default_handler   IRQ_Handler16
  def_default_handler   IRQ_Handler17
  def_default_handler   IRQ_Handler18
  def_default_handler   IRQ_Handler19
  def_default_handler   IRQ_Handler20
  def_default_handler   IRQ_Handler21
  def_default_handler   IRQ_Handler22
  def_default_handler   IRQ_Handler23
  def_default_handler   IRQ_Handler24
  def_default_handler   IRQ_Handler25
  def_default_handler   IRQ_Handler26
  def_default_handler   IRQ_Handler27
  def_default_handler   IRQ_Handler28
  def_default_handler   IRQ_Handler29
  def_default_handler   IRQ_Handler30
  def_default_handler   IRQ_Handler31

  .end

