# file:        bin.mak
# description: Build VLFeat command line utilities
# author:      Andrea Vedaldi


all: bin-all
clean: bin-clean
archclean: bin-archclean
distclean: bin-distclean
info: bin-info

# --------------------------------------------------------------------
#                                                        Configuration
# --------------------------------------------------------------------

BIN_CFLAGS = $(filter-out -std=c99,$(CFLAGS))
BIN_LDFLAGS = $(LDFLAGS) -L$(BINDIR) -lvl

DSP_TOOLS := /opt/TI/TI_CGT_C6000_7.2.2
DSP_DOFFBUILD := /opt/doffbuild
L6X := $(DSP_TOOLS)/bin/cl6x
LNK6X := $(DSP_TOOLS)/bin/lnk6x
DLLCREATE := $(DSP_DOFFBUILD)/bin/DLLcreate
# --------------------------------------------------------------------
#                                                                Build
# --------------------------------------------------------------------

# On Mac OS X the library install_name is prefixed with @loader_path/.
# At run time this causes the loader to search for a local copy of the
# library for any binary which is linked against it. The install_name
# can be modified later by install_name_tool.


#helloworld.x64P: helloworld_dsp.o64P helloworld_bridge.o64P

CFLAGS :=
INCLUDES := -I$(DSP_TOOLS)/include

bin_src := $(wildcard $(VLDIR)/src/progs/dsp/*.c)
bin_tgt := $(addprefix $(BINDIR)/, $(patsubst %.c,%.dll64P,$(notdir $(bin_src))))

all: $(bin_tgt)

#clean:
#	$(QUIET_CLEAN)$(RM) $(bins) *.o *.o64P *.x64P

# pretty print

ifdef V
QUIET_CC    = @echo '   CC         '$@;
QUIET_LINK  = @echo '   LINK       '$@;
QUIET_CLEAN = @echo '   CLEAN      '$@;
QUIET_DLL   = @echo '   DLLCREATE  '$@;
endif

%.o64P:: %.s
	$(QUIET_CC)$(CL6X) $(CFLAGS) $(INCLUDES) -mv=64p -eo.o64P -c $<

%.o64P: %.c
	$(QUIET_CC)$(CL6X) $(CFLAGS) $(INCLUDES) -mv=64p -eo.o64P -c $<

%.x64P: %.o64P
	$(QUIET_LINK)$(LNK6X) -r -cr --localize='$$bss' -o $@ $+

$(BINDIR)%.dll64P: %.x64P
	$(QUIET_DLL)$(DLLCREATE) $< -o=$@






#bin-archclean: bin-clean

#bin-distclean:

#bin-info:
#	$(call echo-title,Command line utilities)
#	$(call dump-var,bin_src)
#	$(call dump-var,bin_tgt)
#	$(call dump-var,bin_dep)
#	$(call echo-var,BIN_CFLAGS)
#	$(call echo-var,BIN_LDFLAGS)
#	@echo

#deploy: all
#	@echo deploying files to beagleboard ...
#	scp -r $(arch_bins)  ubuntu@beagleboard:/home/ubuntu/tom/

# Local variables:
# mode: Makefile
# End:
