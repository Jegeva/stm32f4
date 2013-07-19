# put your *.o targets here, make should handle the rest!

SRCS = main.c stm32f4xx_it.c system_stm32f4xx.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

###################################################

B := main

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

CFLAGS += -I../STM32F4-Discovery_FW_V1.1.0/Libraries/CMSIS/Include -Iinc -Ilib -Ilib/inc 
CFLAGS += -Ilib/inc/peripherals -DARM_MATH_CM4
#CFLAGS += 

SRCS += lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o) 

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS) 
	$(CC) $(CFLAGS) $^ -o $@ -Llib -L../STM32F4-Discovery_FW_V1.1.0/Libraries/CMSIS/Lib -lCMSISM4lf_math -lstm32f4 -lm
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(SIZE) $(PROJ_NAME).elf

prog: 
	st-flash write $(PROJ_NAME).bin 0x8000000

gdb :
	openocd --file /usr/share/openocd/scripts/board/stm32f4discovery.cfg &
	sleep 1
	echo "target extended localhost:3333\nmonitor reset halt\nmonitor reset_nag enable\nb "$(B)"\nc\n" >.gdbinit
	arm-none-eabi-gdb main.elf && killall openocd

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
