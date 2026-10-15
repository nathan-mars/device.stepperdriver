# Harp StepperDriver firmware - command-line build (macOS/Linux, avr-gcc).
# The Microchip Studio project (StepperDriver.cproj) is still valid; this Makefile
# mirrors its Debug configuration flags so both produce equivalent binaries.
#
#   make [HW=1.1]     build  -> build/StepperDriver.hex and the Harp-named release copy
#                     (build/StepperDriver-fw<x.y>-harp<core>-hw<x.y>-ass<n>.hex), print sizes
#   make test         run the host-side unit tests in ../tests (needs clang or gcc)
#   make flash PORT=/dev/tty.usbserial-XXXX   flash with Harp Toolkit over USB
#   make clean        delete the build outputs (explicit list, no wildcards)
#
# Toolchain: Homebrew `avr-gcc@15` is keg-only; override AVR_BIN if yours lives elsewhere.

AVR_BIN  ?= /opt/homebrew/opt/avr-gcc@15/bin
CC        = $(AVR_BIN)/avr-gcc
OBJCOPY  ?= avr-objcopy
OBJDUMP  ?= avr-objdump
SIZE     ?= avr-size

MCU       = atxmega128a1u
F_CPU     = 32000000UL
TARGET    = StepperDriver
BUILD     = build
CORELIB   = ATxmega128A1U-1.15

SRCS = app.c app_funcs.c app_ios_and_regs.c i2c.c interrupts.c main.c motion.c \
       quick_movement.c regs_reset_and_init.c stepper_control.c
OBJS = $(SRCS:%.c=$(BUILD)/%.o)
DEPS = $(OBJS:.o=.d)

# Same options as the .cproj (unsigned char/bitfields, packed structs, short enums,
# relaxed branches). -std=gnu99 matches the dialect the old Atmel toolchain used.
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -g2 -std=gnu99 -Wall \
          -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
          -mrelax -MMD -MP -I.
LDFLAGS = -mmcu=$(MCU) -mrelax -L. -l$(CORELIB) -lm

ELF = $(BUILD)/$(TARGET).elf
HEX = $(BUILD)/$(TARGET).hex
LSS = $(BUILD)/$(TARGET).lss

# Hardware revision the image is built for: 1.0 (default) or 1.1. It is reported by the
# device and checked by Harp Toolkit against the file name, so build per board revision:
#   make HW=1.1
HW      ?= 1.0
HW_H     = $(word 1,$(subst ., ,$(HW)))
HW_L     = $(word 2,$(subst ., ,$(HW)))
CFLAGS  += -DHW_VERSION_H=$(HW_H) -DHW_VERSION_L=$(HW_L)

# Harp release name: <device>-fw<firmware>-harp<core>-hw<hardware>-ass<assembly>.hex
# Firmware/assembly versions are read from app.c, the core version from the library file
# name, so the name can never disagree with what the device reports.
ver     = $(shell sed -n 's/.*uint8_t $(1) *= *\([0-9]*\);.*/\1/p' app.c | head -1)
FW_VER  = $(call ver,fwH).$(call ver,fwL)
HW_VER  = $(HW)
ASS_VER = $(call ver,ass)
CORE_VER = $(shell echo $(CORELIB) | sed -n 's/.*-\([0-9]*\.[0-9]*\).*/\1/p')
RELEASE_HEX = $(BUILD)/$(TARGET)-fw$(FW_VER)-harp$(CORE_VER)-hw$(HW_VER)-ass$(ASS_VER).hex

.PHONY: all size clean test flash

all: $(HEX) $(RELEASE_HEX) $(LSS) size

$(RELEASE_HEX): $(HEX)
	cp $(HEX) $@
	@echo "release image: $@"

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJS) lib$(CORELIB).a
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(LSS): $(ELF)
	$(OBJDUMP) -h -S $< > $@

# ATxmega128A1U: 8192 bytes SRAM. data+bss must leave room for the stack.
size: $(ELF)
	@$(SIZE) $(ELF)
	@$(SIZE) $(ELF) | awk 'NR==2 { used=$$2+$$3; printf "SRAM used by data+bss: %d of 8192 bytes (%d left for stack)\n", used, 8192-used; if (8192-used < 300) { print "ERROR: less than 300 bytes left for the stack"; exit 1 } }'

test:
	$(MAKE) -C ../tests

# Harp Toolkit: `dotnet tool install -g Harp.Toolkit`. dotnet puts it in ~/.dotnet/tools,
# which is often not on PATH, so fall back to that location. The tool targets the .NET 8
# runtime; DOTNET_ROLL_FORWARD=Major lets a newer installed runtime (e.g. .NET 10) run it.
HARP_TOOLKIT ?= $(shell command -v harp.toolkit 2>/dev/null || echo $(HOME)/.dotnet/tools/harp.toolkit)

flash: $(RELEASE_HEX)
	@test -n "$(PORT)" || { echo "usage: make flash PORT=/dev/tty.usbserial-XXXX"; exit 1; }
	@test -x "$(HARP_TOOLKIT)" || { echo "harp.toolkit not found; run: dotnet tool install -g Harp.Toolkit"; exit 1; }
	DOTNET_ROLL_FORWARD=Major "$(HARP_TOOLKIT)" update $(RELEASE_HEX) --port $(PORT)

clean:
	rm -f $(OBJS) $(DEPS) $(ELF) $(HEX) $(RELEASE_HEX) $(LSS)
	-rmdir $(BUILD) 2>/dev/null

-include $(DEPS)
