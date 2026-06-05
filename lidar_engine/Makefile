# Build for Raspberry Pi 5 (Cortex-A76). Requires liblgpio-dev.
#   sudo apt install liblgpio-dev
CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -mcpu=cortex-a76 -std=gnu11 -pthread
LDFLAGS = -llgpio -lpthread -lm

OBJS = globals.o rt_utils.o mathlut.o i2c.o encoder.o imu.o \
       motor.o lidar.o scanner.o thermal.o network.o watchdog.o main.o

TARGET = lidar_engine

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
