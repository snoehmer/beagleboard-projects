DSP_TOOLS := /opt/TI/TI_CGT_C6000_7.2.2
DSP_DOFFBUILD := /opt/doffbuild

CROSS_COMPILE := /usr/local/sourcery-arm-linux/bin/arm-none-linux-gnueabi-
CC := $(CROSS_COMPILE)gcc

CFLAGS := -O2 -Wall -Wextra -Wno-unused-parameter -ansi -std=c99
LDFLAGS := -Wl,--as-needed

override CFLAGS += -D_GNU_SOURCE

DSP_API := 2

override CFLAGS += -DDSP_API=$(DSP_API)

override CFLAGS += -DDEBUG

#ifdef DEBUG
#  override CFLAGS += -DDEBUG
#endif

CL6X := $(DSP_TOOLS)/bin/cl6x
LNK6X := $(DSP_TOOLS)/bin/lnk6x
DLLCREATE := $(DSP_DOFFBUILD)/bin/DLLcreate

#ifdef NEW
#  override CFLAGS += -DNEW_API
#endif

all:

# helloworld

helloworld: helloworld_arm.o dsp_bridge.o log.o

bins += helloworld

helloworld.x64P: helloworld_dsp.o64P helloworld_bridge.o64P

helloworld.dll64P: helloworld.x64P
helloworld.dll64P: CFLAGS :=
helloworld.dll64P: INCLUDES := -I$(DSP_TOOLS)/include

bins += helloworld.dll64P

all: $(bins)

clean:
	$(QUIET_CLEAN)$(RM) $(bins) *.o *.o64P *.x64P

# pretty print
ifndef V
QUIET_CC    = @echo '   CC         '$@;
QUIET_LINK  = @echo '   LINK       '$@;
QUIET_CLEAN = @echo '   CLEAN      '$@;
QUIET_DLL   = @echo '   DLLCREATE  '$@;
endif

%.o64P:: %.s
	$(QUIET_CC)$(CL6X) $(CFLAGS) $(INCLUDES) -mv=64p -eo.o64P -c $<

%.o64P:: %.c
	$(QUIET_CC)$(CL6X) $(CFLAGS) $(INCLUDES) -mv=64p -eo.o64P -c $<

%.x64P::
	$(QUIET_LINK)$(LNK6X) -r -cr --localize='$$bss' -o $@ $+

%.dll64P::
	$(QUIET_DLL)$(DLLCREATE) $< -o=$@

%.o:: %.c
	$(QUIET_CC)$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

helloworld:
	$(QUIET_CC)$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)
