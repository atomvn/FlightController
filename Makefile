
# toolchain
TOOLCHAIN    = arm-none-eabi-
CC           = $(TOOLCHAIN)gcc
CP           = $(TOOLCHAIN)objcopy
AS           = $(TOOLCHAIN)gcc -x assembler-with-cpp
HEX          = $(CP) -O ihex
BIN          = $(CP) -O binary -S

# define mcu, specify the target processor
MCU          = cortex-m3

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJECT_NAME=FlightController

# specify define
DDEFS       =

# define root dir
ROOT_DIR     = .

# define include dir
INCLUDE_DIRS = 

# define build dir
BUILD_DIR = build

# define stm32f10x lib dir
LIBOPENCM3_DIR      = $(ROOT_DIR)/lib/libopencm3

# define freertos dir
FREERTOS_DIR = $(ROOT_DIR)/lib/freertos

# define user dir
SRC_DIR     = $(ROOT_DIR)/src

# link file
LINK_SCRIPT  = $(ROOT_DIR)/config/stm32_flash.ld

# user specific
SRC       =
ASM_SRC   =
SRC      += $(wildcard $(SRC_DIR)/**/*.c)
SRC      += $(wildcard $(SRC_DIR)/app/**/*.c)
SRC      += $(wildcard $(SRC_DIR)/drivers/**/*.c)
SRC      += $(wildcard $(SRC_DIR)/system/**/*.c)

# user include
INCLUDE_DIRS  = $(SRC_DIR)

# include sub makefiles
# include makefile_std_lib.mk   # STM32 Standard Peripheral Library
include ./config/makefile_freertos.mk  # freertos source

INC_DIR  = $(patsubst %, -I%, $(INCLUDE_DIRS))

# run from Flash
DEFS	 = $(DDEFS) -DRUN_FROM_FLASH=1

OBJECTS  = $(ASM_SRC:.s=.o) $(SRC:.c=.o)

# Define optimisation level here
OPT = -Os

MC_FLAGS = -mcpu=$(MCU)

AS_FLAGS = $(MC_FLAGS) -g -gdwarf-2 -mthumb  -Wa,-amhls=$(<:.s=.lst)
CP_FLAGS = $(MC_FLAGS) $(OPT) -g -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
LD_FLAGS = $(MC_FLAGS) -g -gdwarf-2 -mthumb -nostartfiles -Xlinker --gc-sections -T$(LINK_SCRIPT) -Wl,-Map=$(BUILD_DIR)/$(PROJECT_NAME).map,--cref,--no-warn-mismatch

#
# makefile rules
#
all: $(OBJECTS) $(BUILD_DIR)/$(PROJECT_NAME).elf  $(BUILD_DIR)/$(PROJECT_NAME).hex $(BUILD_DIR)/$(PROJECT_NAME).bin
	$(TOOLCHAIN)size $(BUILD_DIR)/$(PROJECT_NAME).elf

%.o: %.c
	$(CC) -c $(CP_FLAGS) -I . $(INC_DIR) $< -o $@

%.o: %.s
	$(AS) -c $(AS_FLAGS) $< -o $@

%.elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LD_FLAGS) -o $@

%.hex: %.elf
	$(HEX) $< $@

%.bin: %.elf
	$(BIN)  $< $@

flash: $(PROJECT_NAME).bin
	st-flash write $(PROJECT_NAME).bin 0x8000000

erase:
	st-flash erase

clean:
	-rm -rf $(OBJECTS)
	-rm -rf $(BUILD_DIR)/$(PROJECT_NAME).elf
	-rm -rf $(BUILD_DIR)/$(PROJECT_NAME).map
	-rm -rf $(BUILD_DIR)/$(PROJECT_NAME).hex
	-rm -rf $(BUILD_DIR)/$(PROJECT_NAME).bin
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(ASM_SRC:.s=.lst)

