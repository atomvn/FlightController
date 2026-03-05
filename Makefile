###############################
#########  Toolchain  #########
###############################
TOOLCHAIN    := arm-none-eabi-
CC           := $(TOOLCHAIN)gcc
OBJCOPY      := $(TOOLCHAIN)objcopy
SIZE         := $(TOOLCHAIN)size

###############################
##########  TARGET  ###########
###############################
MCU          := cortex-m3
PROJECT_NAME := FlightController
LIBNAME	     := opencm3_stm32f1
DEFS         += -DSTM32F1

###############################
########  Directories  ########
###############################
ROOT_DIR     := .
SRC_DIR		 := $(ROOT_DIR)/src
CONFIG_DIR   := $(ROOT_DIR)/config
LIB_DIR      := $(ROOT_DIR)/lib
BUILD_DIR    := $(ROOT_DIR)/build

OPENCM3_DIR  := $(LIB_DIR)/libopencm3
FREERTOS_DIR := $(LIB_DIR)/freertos

LINK_SCRIPT  := $(CONFIG_DIR)/stm32f1_linker.ld

###############################
#######  Make configs  ########
###############################
GENERATE_LST ?= 0
DEBUG        ?= 0

###############################
#######  Source files  ########
###############################
SRC          := $(shell find $(SRC_DIR) -name "*.c")

###############################
#######  Include paths  #######
###############################
INCLUDE_DIRS := $(SRC_DIR)
INCLIDE_DIRS += OPENCM3_DIR/include
include $(CONFIG_DIR)/makefile_freertos.mk

###############################
###########  Flags  ###########
###############################
FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

INC_FLAGS    := $(addprefix -I, $(INCLUDE_DIRS))

DEBUG_FLAGS  := -g -gdwarf-2

COMMON_FLAGS += -ffunction-sections -fdata-sections
COMMON_FLAGS += -Wall
ifeq ($(DEBUG),1)
COMMON_FLAGS += $(DEBUG_FLAGS)
endif

CFLAGS       := $(COMMON_FLAGS) -Os -g -MMD -MP
CFLAGS	     += $(ARCH_FLAGS)
CFLAGS       += -I$(OPENCM3_DIR)/include

ifeq ($(GENERATE_LST),1)
$(BUILD_DIR)/%.o: CFLAGS += -Wa,-ahlms=$(@:.o=.lst)
endif

LDFLAGS      := $(COMMON_FLAGS)
LDFLAGS      += $(ARCH_FLAGS)
LDFLAGS      += -T$(LINK_SCRIPT)
LDFLAGS	  	 += -L$(OPENCM3_DIR)/lib
LDFLAGS      += -Wl,--gc-sections
LDFLAGS      += -Wl,-Map=$(BUILD_DIR)/$(PROJECT_NAME).map,--cref,--no-warn-mismatch
LDFLAGS	     += --static -nostartfiles

DEFINES      := -DRUN_FROM_FLASH=1

###############################
#######  Object files  ########
###############################
OBJ_FILES    := $(patsubst $(ROOT_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRC))

DEPS         := $(OBJ_FILES:.o=.d)

###############################
###########  Rules  ###########
###############################
all: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(SIZE) $<

$(OPENCM3_DIR)/lib/lib$(LIBNAME).a:
	$(MAKE) -C $(OPENCM3_DIR)

$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJ_FILES) $(OPENCM3_DIR)/lib/lib$(LIBNAME).a
	@mkdir -p $(BUILD_DIR)
	$(CC) $^ $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: $(ROOT_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(DEFINES) $(INC_FLAGS) -c $< -o $@

hex: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(OBJCOPY) -O ihex $< $(BUILD_DIR)/$(PROJECT_NAME).hex

bin: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(OBJCOPY) -O binary $< $(BUILD_DIR)/$(PROJECT_NAME).bin

flash: bin
	st-flash write $(BUILD_DIR)/$(PROJECT_NAME).bin 0x08000000

erase:
	st-flash erase

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)

.PHONY: all clean flash erase hex bin
