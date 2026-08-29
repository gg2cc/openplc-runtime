SDK_ROOT_PATH ?= /home/ld/RK3562/rk3562_linux_sdk_release_v1.2.0
SYSROOT ?= /home/ld/RK3562/sysroots/armv8a-linux
CROSS_COMPILE ?= $(SDK_ROOT_PATH)/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
STRIP = $(CROSS_COMPILE)strip

CPPFLAGS += -I../../../../lib -I../../../../drivers -I.. -I../cjson -I. -I./libs/CANopenNode -I./libs/CANopenLinux -I$(SYSROOT)/usr/include \
	-DCO_CONFIG_NMT='(CO_CONFIG_NMT_MASTER | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)' \
	-DCO_CONFIG_HB_CONS='(CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)' \
	-DCO_CONFIG_SDO_CLI='(CO_CONFIG_SDO_CLI_ENABLE | CO_CONFIG_SDO_CLI_SEGMENTED | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)' \
	-DCO_CONFIG_GTW='(CO_CONFIG_GTW_ASCII | CO_CONFIG_GTW_ASCII_SDO | CO_CONFIG_GTW_ASCII_NMT | CO_CONFIG_GTW_ASCII_LSS | CO_CONFIG_GTW_ASCII_LOG | CO_CONFIG_GTW_ASCII_ERROR_DESC | CO_CONFIG_GTW_ASCII_PRINT_HELP | CO_CONFIG_GTW_ASCII_PRINT_LEDS)' \
	-DCO_CONFIG_GTW_BLOCK_DL_LOOP=3 \
	-DCO_CONFIG_GTWA_COMM_BUF_SIZE=2000 \
	-DCO_CONFIG_GTWA_LOG_BUF_SIZE=10000 \
	-DCO_CONFIG_FIFO='(CO_CONFIG_FIFO_ENABLE | CO_CONFIG_FIFO_ALT_READ | CO_CONFIG_FIFO_CRC16_CCITT | CO_CONFIG_FIFO_ASCII_COMMANDS | CO_CONFIG_FIFO_ASCII_DATATYPES)'
CFLAGS += -O2 -fPIC -shared -Wall -Wextra -Wno-unused-parameter -Wno-format-truncation -Wno-stringop-overflow
LDFLAGS += -L$(SYSROOT)/usr/lib -L$(SYSROOT)/usr/lib64 -Wl,-rpath-link,$(SYSROOT)/usr/lib -Wl,-rpath-link,$(SYSROOT)/usr/lib64 -lpthread

TARGET        = libcanopen_plugin.so
SOURCES       = canopen_plugin.c canopen_config.c OD.c ../plugin_logger.c ../cjson/cJSON.c ../can/can_socket.c ../can/can_netlink.c \
	./libs/CANopenLinux/CO_driver.c ./libs/CANopenLinux/CO_error.c ./libs/CANopenLinux/CO_epoll_interface.c
CANOPEN_NODE_SOURCES = \
	./libs/CANopenNode/301/CO_ODinterface.c \
	./libs/CANopenNode/301/CO_fifo.c \
	./libs/CANopenNode/301/CO_NMT_Heartbeat.c \
    ./libs/CANopenNode/301/CO_Node_Guarding.c \
	./libs/CANopenNode/301/CO_HBconsumer.c \
	./libs/CANopenNode/301/CO_Emergency.c \
	./libs/CANopenNode/301/CO_SDOclient.c \
	./libs/CANopenNode/301/CO_SDOserver.c \
	./libs/CANopenNode/301/CO_TIME.c \
	./libs/CANopenNode/301/CO_SYNC.c \
	./libs/CANopenNode/301/CO_PDO.c \
    ./libs/CANopenNode/301/crc16-ccitt.c \
	./libs/CANopenNode/303/CO_LEDs.c \
	./libs/CANopenNode/305/CO_LSSslave.c \
    ./libs/CANopenNode/305/CO_LSSmaster.c \
	./libs/CANopenNode/309/CO_gateway_ascii.c \
	./libs/CANopenNode/storage/CO_storage.c \
	./libs/CANopenNode/CANopen.c
all: $(TARGET)

$(TARGET): $(SOURCES) $(CANOPEN_NODE_SOURCES)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $(TARGET) $(SOURCES) $(CANOPEN_NODE_SOURCES) $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean

# make -C core/src/drivers/plugins/native/canopen
# ./core/src/deploy-to-3562.sh
# nm -D build/plugins/libcanopen_plugin.so | grep start_loop
