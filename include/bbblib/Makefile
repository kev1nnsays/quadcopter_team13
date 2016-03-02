#
# Makefile for modified BBB library:  
# TODO:  integrate with Pixy/LCM library
#
CC=gcc
HEAD=bbb.h dynamixel.h
SRCS=bbb_adc.c bbb_gpio.c bbb_i2c.c bbb_init.c bbb_pwm.c
OBJS=$(SRCS:.c=.o)

all: $(OBJS) bbb_test_ADC bbb_test_GPIO bbb_test_PWM bbb_test_I2C bbb_lib bbb_test_dynamixel

bbb_test_ADC: bbb_adc.o bbb_init.o bbb_test_ADC.o
	$(CC) bbb_adc.o bbb_init.o bbb_test_ADC.o -o bbb_test_ADC

bbb_test_GPIO: bbb_gpio.o bbb_init.o bbb_test_GPIO.o
	$(CC) $^ -o bbb_test_GPIO

bbb_test_PWM: bbb_pwm.o bbb_init.o bbb_test_PWM.o 
	$(CC) $^ -o bbb_test_PWM

bbb_test_I2C: bbb_i2c.o bbb_init.o bbb_test_I2C.o
	$(CC) $^ -o bbb_test_I2C

bbb_test_dynamixel: bbb_uart.o bbb_dynamixel.o bbb_init.o bbb_gpio.o bbb_test_dynamixel.o;\
    $(CC) $^ -O3 -o bbb_test_dynamixel -lpthread
	
bbb_lib: bbb_init.o bbb_gpio.o bbb_adc.o bbb_pwm.o bbb_i2c.o
	ar rcs libbbb.a $^

# Default compile rule
%.o: %.c $(HEAD)
	$(CC) -O3 -c -o $@ $<

clean:
	/bin/rm -f *.o bbb_test_ADC bbb_test_GPIO bbb_test_PWM bbb_test_I2C bbb_test_dynamixel
