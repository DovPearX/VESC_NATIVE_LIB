CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
OBJDUMP = arm-none-eabi-objdump
OBJCOPY = arm-none-eabi-objcopy
PYTHON = python3

STLIB_PATH = $(VESC_C_LIB_PATH)/stdperiph_stm32f4/
BUILD_DIR = build

ifeq ($(USE_STLIB),yes)
	SOURCES += \
		$(STLIB_PATH)/src/misc.c \
		$(STLIB_PATH)/src/stm32f4xx_adc.c \
		$(STLIB_PATH)/src/stm32f4xx_dma.c \
		$(STLIB_PATH)/src/stm32f4xx_exti.c \
		$(STLIB_PATH)/src/stm32f4xx_flash.c \
		$(STLIB_PATH)/src/stm32f4xx_rcc.c \
		$(STLIB_PATH)/src/stm32f4xx_syscfg.c \
		$(STLIB_PATH)/src/stm32f4xx_tim.c \
		$(STLIB_PATH)/src/stm32f4xx_iwdg.c \
		$(STLIB_PATH)/src/stm32f4xx_wwdg.c
endif

UTILS_PATH = $(VESC_C_LIB_PATH)/utils/

SOURCES += $(UTILS_PATH)/rb.c
SOURCES += $(UTILS_PATH)/utils.c

OBJECTS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(SOURCES))

ifeq ($(USE_OPT),)
	USE_OPT =
endif

CFLAGS = -fpic -Os -Wall -Wextra -Wundef -std=gnu99 -I$(VESC_C_LIB_PATH)
CFLAGS += -I$(STLIB_PATH)/CMSIS/include -I$(STLIB_PATH)/CMSIS/ST -I$(UTILS_PATH)/
CFLAGS += -fomit-frame-pointer -falign-functions=16 -mthumb
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -DIS_VESC_LIB
CFLAGS += $(USE_OPT)

ifeq ($(USE_STLIB),yes)
	CFLAGS += -DUSE_STLIB -I$(STLIB_PATH)/inc
endif

LDFLAGS = -nostartfiles -static -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4
LDFLAGS += -lm -Wl,--gc-sections,--undefined=init
LDFLAGS += -T $(VESC_C_LIB_PATH)/link.ld

.PHONY: default all clean clean-obj

default: $(BUILD_DIR)/$(TARGET)
all: default

# Create the build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Create the build directory for object files if it doesn't exist
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(BUILD_DIR)/$(TARGET) $(OBJECTS)

# Ensure build directory is created before linking
$(BUILD_DIR)/$(TARGET): $(OBJECTS) | $(BUILD_DIR)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $(BUILD_DIR)/$(TARGET).elf
	$(OBJDUMP) -D $(BUILD_DIR)/$(TARGET).elf > $(BUILD_DIR)/$(TARGET).list
	$(OBJCOPY) -O binary $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin --gap-fill 0x00
	$(PYTHON) $(VESC_C_LIB_PATH)/conv.py -f $(BUILD_DIR)/$(TARGET).bin -n $(BUILD_DIR)/$(TARGET) > $(BUILD_DIR)/$(TARGET).lisp
	$(MAKE) clean-obj

clean-obj:
	find $(BUILD_DIR) -type f -name '*.o' -delete
	find $(BUILD_DIR) -type d -empty -delete

clean:
	rm -rf $(BUILD_DIR) $(ADD_TO_CLEAN)
