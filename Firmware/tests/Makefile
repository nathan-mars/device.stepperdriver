# Host-side unit tests for the motion engine. Needs clang or gcc.
CC     ?= cc
CFLAGS  = -std=c99 -O1 -g -Wall -Wextra -Wno-unused-parameter -I. -I../StepperDriver -fsanitize=address,undefined
SRCS    = test_motion.c sim_hal.c ../StepperDriver/motion.c
BUILD   = build

.PHONY: all test clean
all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_motion: $(SRCS) ../StepperDriver/motion.h motion_port.h sim_hal.h | $(BUILD)
	$(CC) $(CFLAGS) $(SRCS) -o $@ -lm

test: $(BUILD)/test_motion
	./$(BUILD)/test_motion

clean:
	rm -f $(BUILD)/test_motion $(BUILD)/profile.csv
	-rmdir $(BUILD) 2>/dev/null
