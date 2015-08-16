CC ?= gcc
HIDAPI_CFLAGS := $(shell pkg-config --cflags hidapi-hidraw)
HIDAPI_LIBS := $(shell pkg-config --libs hidapi-hidraw)
CFLAGS ?= -O2 -Wall
KDIR ?= /lib/modules/$(shell uname -r)/build
PREFIX ?= /usr/local
OS_NAME := $(shell uname -s)

ifeq ($(OS_NAME),Linux)
    HWMON_TARGET = hwmon
    HWMON_INSTALL_TARGET = hwmon_install
    HWMON_CLEAN_TARGET = hwmon_clean
else
    HWMON_TARGET =
    HWMON_INSTALL_TARGET = hwmon_install
    HWMON_CLEAN_TARGET = hwmon_clean
endif

.PHONY: hwmon hwmon_install hwmon_clean

all: zdpms-config $(HWMON_TARGET)

install: all $(HWMON_INSTALL_TARGET)
	mkdir -p ${DESTDIR}${PREFIX}/bin
	install -m 0755 zdpms-config ${DESTDIR}${PREFIX}/bin

zdpms-config: zdpms-config.o
	$(CC) $^ $(HIDAPI_LIBS) $(LDFLAGS) -o $@

zdpms-config.o: zdpms-config.c
	$(CC) $(HIDAPI_CFLAGS) $(CFLAGS) $(CPPFLAGS) -D_GNU_SOURCE -c $^ -o $@

clean: $(HWMON_CLEAN_TARGET)
	rm -f *.o zdpms-config

hwmon:
	$(MAKE) -C ${KDIR} M=${PWD}/hwmon modules

hwmon_clean:
	$(MAKE) -C ${KDIR} M=${PWD}/hwmon clean

hwmon_install:
	$(MAKE) -C ${KDIR} INSTALL_MOD_PATH=${DESTDIR} M=${PWD}/hwmon modules_install
	if [ "$$UID" = 0 ] && [ -z "${DESTDIR}" ]; then depmod -a; fi

