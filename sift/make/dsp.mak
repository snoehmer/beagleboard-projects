# file:        dsp.mak
# description: Build VLFeat command line utilities
# author:      Andrea Vedaldi


all: dsp-bin-all
clean: dsp-bin-clean
archclean: bin-archclean
distclean: bin-distclean
info: bin-info

# --------------------------------------------------------------------
#                                                        Configuration
# --------------------------------------------------------------------


DSP_TOOLS := /opt/TI/TI_CGT_C6000_7.2.2
DSP_DOFFBUILD := /opt/doffbuild
CL6X := $(DSP_TOOLS)/bin/cl6x
LNK6X := $(DSP_TOOLS)/bin/lnk6x
DLLCREATE := $(DSP_DOFFBUILD)/bin/DLLcreate
# --------------------------------------------------------------------
#                                                                Build
# --------------------------------------------------------------------

# On Mac OS X the library install_name is prefixed with @loader_path/.
# At run time this causes the loader to search for a local copy of the
# library for any binary which is linked against it. The install_name
# can be modified later by install_name_tool.


DSPCFLAGS := 
DSPINCLUDES := -I$(DSP_TOOLS)/include/

dsp_bin_src_dir := $(VLDIR)/src/progs/dsp
dsp_bin_src := $(wildcard $(dsp_bin_src_dir)/*.c)
dsp_bin_tgt := $(addprefix $(BINDIR)/, $(patsubst %.c,%.dll64P,$(notdir $(dsp_bin_src))))

dsp-bin-all: $(dsp_bin_tgt)

arch_bins += $(dsp_bin_tgt)

dsp-bin-clean:
	$(QUIET_CLEAN)$(RM) $(dsp_bin_tgt) $(BINDIR)/objs/libdsp/*
	$(QUIET_CLEAN)$(RM) $(dsp_bin_tgt) $(BINDIR)/objs/*.o64P

# pretty print

ifndef V
QUIET_CC    = @echo '   CC         '$@;
QUIET_LINK  = @echo '   LINK       '$@;
QUIET_CLEAN = @echo '   CLEAN      '$@;
QUIET_DLL   = @echo '   DLLCREATE  '$@;
endif



$(BINDIR)/objs/libdsp/%.o64P: $(dsp_bin_src_dir)/%.c $(BINDIR)/objs/libdsp/.dirstamp
	$(QUIET_CC)$(CL6X) $(DSPCFLAGS) $(DSPINCLUDES) -mv=64p -eo.o64P --obj_directory $(dir $@) -c $<	


$(BINDIR)/objs/libdsp/%_bridge.o64P: $(dsp_bin_src_dir)/%_bridge.s
	$(QUIET_CC)$(CL6X) $(DSPCFLAGS) $(DSPINCLUDES) -mv=64p -eo.o64P --obj_directory $(dir $@) -c $<	

$(BINDIR)/objs/libdsp/%.x64P: $(BINDIR)/objs/libdsp/%.o64P $(BINDIR)/objs/libdsp/%_bridge.o64P $(dsp_dll_obj)
	$(QUIET_LINK)$(LNK6X) -r -cr --localize='$$bss' -o $@ $+


$(BINDIR)/%.dll64P: $(BINDIR)/objs/libdsp/%.x64P
	$(QUIET_DLL)$(DLLCREATE) $< -o=$@

#################        Vlfeat-library    ############################


dsp_dll_src := $(VLDIR)/vl/sift.c $(VLDIR)/vl/host.c $(VLDIR)/vl/generic.c $(VLDIR)/vl/random.c $(VLDIR)/vl/imopv.c $(VLDIR)/vl/mathop.c 
dsp_dll_obj := $(addprefix $(BINDIR)/objs/, $(notdir $(dsp_dll_src:.c=.o64P)))

DSPCFLAGS_VL := -gcc -DVL_DISABLE_THREADS -DVL_DISABLE_SSE2 -I$(VLDIR) -DARM_DSP

dsp_vl_obj: $(dsp_dll_obj)

$(BINDIR)/objs/%.o64P: $(VLDIR)/vl/%.c $(BINDIR)/objs/.dirstamp
	@echo $+
	$(QUIET_CC)$(CL6X) $(DSPCFLAGS_VL) $(DSPINCLUDES) -mv=64p -eo.o64P --obj_directory $(dir $@) -c $< 






#######################################################################







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
