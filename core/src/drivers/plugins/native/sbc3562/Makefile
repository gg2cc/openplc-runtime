SDK_ROOT_PATH ?= /home/ld/RK3562/rk3562_linux_sdk_release_v1.2.0
SYSROOT ?= /home/ld/RK3562/sysroots/armv8a-linux
CROSS_COMPILE ?= $(SDK_ROOT_PATH)/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
STRIP = $(CROSS_COMPILE)strip

CPPFLAGS += -I../../../../lib -I../../../../drivers -I.. -I$(SYSROOT)/usr/include
CFLAGS += -O2 -fPIC -shared -Wall -Wextra -Wno-format-truncation
LDFLAGS += -L$(SYSROOT)/usr/lib -L$(SYSROOT)/usr/lib64 -Wl,-rpath-link,$(SYSROOT)/usr/lib -Wl,-rpath-link,$(SYSROOT)/usr/lib64 -lpthread

TARGET        = huiz_3562_io.so
SOURCE        = huiz_3562_io.c
PLUGIN_LOGGER = ../plugin_logger.c

all: $(TARGET)

$(TARGET): $(SOURCE) $(PLUGIN_LOGGER)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $(TARGET) $(SOURCE) $(PLUGIN_LOGGER) $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean

# make -C core/src/drivers/plugins/native/sbc3562
