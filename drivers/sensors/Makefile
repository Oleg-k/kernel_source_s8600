#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_SENSORS_BMA250)    += bma250_driver.o
obj-$(CONFIG_SENSORS_BMM050)   += bmm050_driver.o bmm050.o
obj-$(CONFIG_OPTICAL_GP2A)	+= gp2a_rpc.o gp2a_proximity.o gp2a_light.o

obj-$(CONFIG_SENSORS_CORE)	+= sensors_core.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

