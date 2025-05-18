# CortexM0_CPU

// Brief description

The implementation of Cortex M0 CPU, which is based on ARMv6-m ISA. It is implemented as 5 stage pipeline.
The design supports full-forwarding and always-taken branch prediction. 

Total 68 instructions are well operated in this implementations. The specific structures of the CPU are given in the report.

This project is based on the course EE511 in KAIST EE division, but since almost program is implemented by myself, I uploaded this on github.
Please note that compiling supports are implemented by KAIST TAs of the course, while other testbenches and most implementations are conducted by me.

// Compile

I assumed that iverilog is used for compile. Downloads all files into your local environment then compile with following instruction.
=> iverilog -c tb.f -o tb

If you execute tb binary file, then you will see that all tests are passed. The example test program (test.c) is based on sorting algorithm.
You can generate other example test program (test.hex) with compile.sh. This program is given by the course. However, note that you must modify the testbench as well.

// Instruction set simulator

Before implementing the CPU, I made an instruction set simulator with c language. Those are included in Sim repository. 
Note that thumb.c and iss.h are made by me, while others are made by the course.
You can simulate instructions in test.hex with run.sh.
