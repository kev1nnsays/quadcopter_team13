# Add your targets (i.e. the name of your output program) here
OBJS = quadcopter_main.o auto_control.o command_handler.o \
       processing_loop.o run_imu.o run_motion_capture.o util.o \
       run_dynamixel_comm.o set_dynamixel.o wp_queue.o wp_trigger_handler.o
IDIR = 	include
_DEPS = quadcopter_main.h quadcopter_struct.h run_imu.h run_motion_capture.h util.h

LCMTYPES = cfg_data_frequency_t.o cfg_uart_baud_t.o cfg_usb_serial_num_t.o channels_t.o \
        kill_t.o waypoint_trigger_t.o pid_status_t.o pose_t.o fsm_state_t.o

BBBOBJS = bbb_init.o bbb_i2c.o bbb_dynamixel.o bbb_uart.o bbb_gpio.o

CC = g++
CC2 = gcc
CFLAGS = -g -Wall `pkg-config --cflags lcm`
LDFLAGS = `pkg-config --libs lcm`

OBJS := $(addprefix src/,$(OBJS))

DEPS := $(addprefix $(IDIR)/,$(_DEPS))
LCMTYPES := $(addprefix include/lcmtypes/lcmtypes_c/,$(LCMTYPES))
BBBOBJS := $(addprefix include/bbblib/,$(BBBOBJS))

.PHONY: all clean io include/lcmtypes include/bbblib

all:  include/lcmtypes include/bbblib $(OBJS) obj/comms_driver obj/quadcopter_main

include/lcmtypes:
	@$(MAKE) -C include/lcmtypes

include/bbblib:
	@$(MAKE) -C include/bbblib

### Basic Make data

blocks/io/%.o: blocks/io/%.c
	$(CC2) $(CFLAGS) -c $^ -I. -o $@

blocks/%.o: blocks/%.c
	$(CC2) $(CFLAGS) -c $^ -I. -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@


include/lcmtypes/%.o: include/lcmtypes/%.c
	$(CC) $(CFLAGS) -c $^ -o $@


### Specific programs to build
#bin/test: $(OBJS)
#	$(CC) $@ $^ $(LDFLAGS) $(LCMTYPES) $(BBBOBJS) -lm -lpthread

obj/comms_driver: blocks/comms_driver.o blocks/io/comms.o blocks/io/serial.o blocks/io/circular.o
	$(CC2) -o $@ $^ -Iio $(LDFLAGS) $(LCMTYPES)

obj/quadcopter_main: $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS) $(LCMTYPES) $(BBBOBJS) -lm -lpthread

#obj/test: $(OBJS)
#	$(CC) -o $@ $^ $(LDFLAGS) $(LCMTYPES) $(BBBOBJS) -lm -lpthread

# Clean target
clean:
	find . -type f | xargs -n 5 touch
	@$(MAKE) -C include/lcmtypes clean
	@$(MAKE) -C include/bbblib clean
	rm -f *~ *.o obj/* src/*~ src/*.o include/*~ blocks/io/*.o blocks/*.o
