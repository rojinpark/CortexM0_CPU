#########################################################
#                                                       #
#  EE511 Midterm Project                                #
#                                                       #
#  Makefile for Test Codes                              #
#                                                       #
#########################################################

# TARGET name Description
TARGET=test

# Cross compile description
CROSS_COMPILE=arm-none-eabi-

# Binary Util description
CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)as
OBJDUMP=$(CROSS_COMPILE)objdump
OBJCOPY=$(CROSS_COMPILE)objcopy

SRCS = test.c
STARTUP = test.s

OBJS = test.o

# Linker script & flags
LDSCRIPT = linkerscript.ld
LDFLAGS = -Wl,--gc-sections -nostartfiles

# C flags
CFLAGS = \
		-Wall \
		-ffunction-sections \
		-T$(LDSCRIPT)\
		-mthumb\
		-mcpu=cortex-m0\

# Top symbol										
ALL: $(TARGET).elf $(TARGET).dis $(TARGET).bin

# Disassemble format
$(TARGET).dis: $(TARGET).bin 
	$(OBJDUMP) -D $(TARGET).elf > $(TARGET).dis 

# Binary format
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) $(TARGET).elf -O binary $(TARGET).bin

# ELF format 
$(TARGET).elf: $(OBJS) $(STARTUP) Makefile
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) $(STARTUP) -o $(TARGET).elf 

# OBJ format
$(OBJS) : %.o : %.c Makefile 
	@echo "*** Compile source codes"
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm *.o $(TARGET).elf $(TARGET).dis $(TARGET).bin $(TARGET).hex
