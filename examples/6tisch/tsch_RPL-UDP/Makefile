CONTIKI_PROJECT = tsch_udp-client tsch_udp-server
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = sky nrf52dk native simplelink

CONTIKI=../../..

 # force Orchestra from command line
MAKE_WITH_ORCHESTRA ?= 0
# force Security from command line
MAKE_WITH_SECURITY ?= 0
 # print #routes periodically, used for regression tests
MAKE_WITH_PERIODIC_ROUTES_PRINT ?= 0

MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/services/shell

#MAKE_ROUTING = MAKE_ROUTING_RPL_CLASSIC

ifeq ($(MAKE_WITH_ORCHESTRA),1)
MODULES += os/services/orchestra
endif

ifeq ($(MAKE_WITH_SECURITY),1)
CFLAGS += -DWITH_SECURITY=1
endif

ifeq ($(MAKE_WITH_PERIODIC_ROUTES_PRINT),1)
CFLAGS += -DWITH_PERIODIC_ROUTES_PRINT=1
endif

include $(CONTIKI)/Makefile.include
