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
obj-$(CONFIG_SENSORS_YDA173)	+= yda173.o
obj-$(CONFIG_SENSORS_YDA165)	+= yda165.o
obj-$(CONFIG_USB_SWITCH_FSA9485) += fsa9485.o
obj-$(CONFIG_USB_SWITCH_MAX14577) += md_max14577.o
ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

