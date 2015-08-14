CC ?= gcc
HIDAPI_CFLAGS := $(shell pkg-config --cflags hidapi-hidraw)
HIDAPI_LIBS := $(shell pkg-config --libs hidapi-hidraw)
CFLAGS ?= -O2 -Wall

all: zdpms-config

zdpms-config: zdpms-config.o
	$(CC) $^ $(HIDAPI_LIBS) $(LDFLAGS) -o $@

zdpms-config.o: zdpms-config.c
	$(CC) $(HIDAPI_CFLAGS) $(CFLAGS) $(CPPFLAGS) -D_GNU_SOURCE -c $^ -o $@

clean:
	rm -f *.o zdpms-config

