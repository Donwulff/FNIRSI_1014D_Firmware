CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
BUILD = build
TARGET = $(BUILD)/bootloader_1014d_base
MKSUNXI = ../fnirsi_101xd_scope/mksunxi

# Keep the original loader's optimization level and peripheral timing.
CFLAGS = -g -O0 -Wall -Wno-write-strings -Wno-char-subscripts \
	-fno-stack-protector -fcommon -DNO_STDLIB=1 -mcpu=arm926ej-s
SOURCES = bl_display_lib.c bl_fonts.c bl_fpga_control.c bl_sd_card_interface.c \
	bl_uart.c bl_variables.c ccu_control.c display_control.c dram_control.c \
	fnirsi_1014d_startup.c spi_control.c
ASSEMBLY = start.s memcmp.s memcpy.s memset.s
OBJECTS = $(SOURCES:%.c=$(BUILD)/%.o) $(ASSEMBLY:%.s=$(BUILD)/%.o)

.PHONY: all clean
.DELETE_ON_ERROR:
all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf $(MKSUNXI) ../tools/check_bootloader.py
	$(OBJCOPY) -O binary --only-section=.text $< $@
	$(MKSUNXI) $@
	python3 ../tools/check_bootloader.py $@

$(TARGET).elf: $(OBJECTS) fnirsi_sd_card_bootloader.ld
	$(CC) -o $@ $(OBJECTS) -Tfnirsi_sd_card_bootloader.ld -nostdlib -lgcc -Wl,-Map,$(TARGET).map

$(BUILD)/%.o: %.c Makefile | $(BUILD)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.s Makefile | $(BUILD)
	$(CC) -mcpu=arm926ej-s -g -x assembler-with-cpp -c $< -o $@

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
