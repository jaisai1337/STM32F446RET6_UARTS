# Toolchain
CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
SIZE = arm-none-eabi-size 
OBJCOPY = arm-none-eabi-objcopy

PROJECT = UARTS

# MCU Options
CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Directories
SRC_DIR = src
SYS_DIR = src/system
DRIVER_DIR = drivers
UTILS_DIR = utils
TEST_CASE_DIR = test_cases
INC_DIR = inc
BUILD_DIR = build

# Sources and objects
SRCS = $(wildcard $(SRC_DIR)/*.c)
SYS_SRCS = $(wildcard $(SYS_DIR)/*.c) $(wildcard $(SYS_DIR)/*.s)
DRIVER_SRCS = $(wildcard $(DRIVER_DIR)/src/*.c)
UTILS_SRCS = $(wildcard $(UTILS_DIR)/src/*.c)
TEST_CASE_SRCS = $(wildcard $(TEST_CASE_DIR)/src/*.c)

OBJS = \
		$(patsubst $(SYS_DIR)/%.s,$(BUILD_DIR)/%.o,$(filter %.s,$(SYS_SRCS))) \
		$(patsubst $(SYS_DIR)/%.c,$(BUILD_DIR)/%.o,$(filter %.c,$(SYS_SRCS))) \
		$(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(filter %.c,$(SRCS))) \
		$(patsubst $(DRIVER_DIR)/src/%.c,$(BUILD_DIR)/%.o,$(filter %.c,$(DRIVER_SRCS))) \
		$(patsubst $(UTILS_DIR)/src/%.c,$(BUILD_DIR)/%.o,$(filter %.c,$(UTILS_SRCS))) \
		$(patsubst $(TEST_CASE_DIR)/src/%.c,$(BUILD_DIR)/%.o,$(filter %.c,$(TEST_CASE_SRCS)))


ELF  = $(BUILD_DIR)/$(PROJECT).elf
BIN  = $(BUILD_DIR)/$(PROJECT).bin
HEX  = $(BUILD_DIR)/$(PROJECT).hex
JLINK = $(BUILD_DIR)/$(PROJECT).jlink
MAP_FILE = $(BUILD_DIR)/$(PROJECT).map 

# Flags
INCLUDES = -I$(INC_DIR) \
	-I$(INC_DIR)/Core/Include \
	-I$(DRIVER_DIR)/inc \
	-I$(UTILS_DIR)/inc \
	-I$(TEST_CASE_DIR)/inc

CFLAGS = $(CPU)  -std=gnu11 -g -Wall -w -Os -ffunction-sections -fdata-sections -MMD -MP $(INCLUDES) -DSTM32F446xx
ASM_FLAGS = $(CPU) -g3 -x assembler-with-cpp
LDFLAGS = -TSTM32F446RETX_FLASH.ld -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs -lc -lnosys -Wl,-Map=$(MAP_FILE) 



# Default target
all: $(BUILD_DIR) $(ELF) $(BIN) $(HEX) $(JLINK)
	@echo "Build complete: $(PROJECT)"

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Assemble S files
$(BUILD_DIR)/%.o: $(SYS_DIR)/%.s
	$(CC) $(ASM_FLAGS) -c $< -o $@

# Compile C files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(SYS_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(DRIVER_DIR)/src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(UTILS_DIR)/src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(TEST_CASE_DIR)/src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link objects to ELF
$(ELF): $(OBJS)
	$(CC) $(CPU) $(OBJS) -o $@ $(LDFLAGS) 
	+@make -s print_size

# Convert ELF to BIN
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

# Convert ELF to HEX
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(JLINK): $(HEX) hex_to_jlink_w4.py
	@echo "Generating J-Link w4 script..."
	python3 hex_to_jlink_w4.py $(HEX) $(JLINK)
	@echo "Done: $(JLINK)"

# Flash using JLink
flash: $(BIN)
	JLinkExe -device STM32F446RE -if SWD -speed 4000 -autoconnect 1 -CommandFile jflash.jlink

manual_flash:
	JLinkExe -device STM32F446RE -if SWD -speed 4000 -CommandFile $(JLINK)
# Clean build directory
clean:
	rm -rf $(BUILD_DIR)


print_size:
	@echo "Memory usage summary:"
	@$(SIZE) -B $(ELF)
	@echo ""
	@$(SIZE) -B -A $(ELF) | awk '\
		/\.isr_vector/  { flash += $$2 } \
		/\.text/        { flash += $$2 } \
		/\.rodata/      { flash += $$2 } \
		/\.ARM.extab/   { flash += $$2 } \
		/\.ARM/         { flash += $$2 } \
		/\.init_array/  { flash += $$2 } \
		/\.fini_array/  { flash += $$2 } \
		/\.data/        { flash += $$2; ram += $$2 } \
		/\.bss/         { ram += $$2 } \
		/_user_heap_stack/ { ram += $$2 } \
		END { \
			flash_total = 524288; \
			ram_total   = 131072; \
			ram_perc   = ram / ram_total * 100; \
			flash_perc = flash / flash_total * 100; \
			ram_bar   = int(ram_perc / 10); \
			flash_bar = int(flash_perc / 10); \
			printf "RAM:   [%s%s] %5.1f%% (used %d of %d bytes)\n", \
				repeat("#", ram_bar), repeat(" ", 10 - ram_bar), \
				ram_perc, ram, ram_total; \
			printf "Flash: [%s%s] %5.1f%% (used %d of %d bytes)\n", \
				repeat("#", flash_bar), repeat(" ", 10 - flash_bar), \
				flash_perc, flash, flash_total; \
		} \
		function repeat(s, n, r) { \
			r = ""; for (i = 0; i < n; i++) r = r s; return r; \
		}'

.PHONY: all flash clean