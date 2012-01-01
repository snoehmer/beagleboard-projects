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



DSP_API := 2
BIN_CFLAGS += -DDSP_API=$(DSP_API) -ansi
BIN_CFLAGS += -D_GNU_SOURCE

ifneq ($(DBG),)
BIN_CFLAGS += -g
endif


# --------------------------------------------------------------------
#                                                                Build
# --------------------------------------------------------------------

# On Mac OS X the library install_name is prefixed with @loader_path/.
# At run time this causes the loader to search for a local copy of the
# library for any binary which is linked against it. The install_name
# can be modified later by install_name_tool.

bin_src := $(wildcard $(VLDIR)/src/progs/arm/*.cpp)
bin_tgt := $(addprefix $(BINDIR)/, $(patsubst %.cpp,%,$(notdir $(bin_src))))
bin_dep := $(addsuffix .d, $(bin_tgt))

lib_src := $(wildcard $(VLDIR)/src/lib/arm/*.cpp)
lib_tgt := $(addprefix $(BINDIR)/objs/libarm/, $(patsubst %.cpp,%.o,$(notdir $(lib_src))))


deps += $(bin_dep)
arch_bins += $(bin_tgt)
comm_bins +=

.PHONY: bin-all, bin-info
.PHONY: bin-clean, bin-archclean, bin-distclean
no_dep_targets += bin-dir bin-clean bin-archclean bin-distclean
no_dep_targets += bin-info

bin-all: $(bin_tgt)

$(BINDIR)/% : $(VLDIR)/src/progs/arm/%.cpp $(dll-dir) $(lib_tgt)
	@make -s $(dll_tgt)
	$(call C,CPP) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(lib_tgt) "$<" -o "$@"

$(BINDIR)/%.d : $(VLDIR)/src/progs/arm/%.cpp $(dll-dir)
	$(call C,CPP) $(BIN_CFLAGS) -M -MT  \
	       '$(BINDIR)/$* $(BINDIR)/$*.d' \
	       "$<" -MF "$@"    

$(BINDIR)/objs/libarm/%.o : $(lib_src) $(dll-dir) $(BINDIR)/objs/libarm/.dirstamp  
	$(call C,CPP) $(BIN_CFLAGS) $(BIN_LDFLAGS) -c "$(VLDIR)/src/lib/arm/$(patsubst %.o,%.cpp,$(notdir $@))" -o $@


bin-clean:
	rm -f $(bin_dep) $(bin_tgt)

bin-archclean: bin-clean

bin-distclean:

bin-info:
	$(call echo-title,Command line utilities)
	$(call dump-var,bin_src)
	$(call dump-var,bin_tgt)
	$(call dump-var,bin_dep)
	$(call echo-var,BIN_CFLAGS)
	$(call echo-var,BIN_LDFLAGS)
	@echo

deploy: all
	@echo deploying files to beagleboard ...
	#scp -r $(arch_bins)  ubuntu@beagleboard:/home/ubuntu/tom/
#	scp -P 23456 -r $(arch_bins)  ubuntu@bierwg.no-ip.org:/home/ubuntu/tom/
#	rsync -avze ssh -P 23456 $(arch_bins) ubuntu@bierwg.no-ip.org:/home/ubuntu/tom
#	rsync -avze --progress --inplace --rsh='ssh -p23456' $(arch_bins) ubuntu@bierwg.no-ip.org:/home/ubuntu/tom
# rsync -avze --progress --inplace --rsh='ssh' $(arch_bins) ubuntu@beagleboard:/home/ubuntu/tom
	rsync -avze --progress --inplace --rsh='ssh' $(arch_bins) ubuntu@noehmer.no-ip.org:/home/ubuntu/tom

deploy_src:
#	rsync -avze --progress --inplace --rsh='ssh -p23456' src/ ubuntu@bierwg.no-ip.org:/home/ubuntu/tom/src
#	rsync -avze --progress --inplace --rsh='ssh -p23456' src/ ubuntu@beagleboard:/home/ubuntu/tom/src
	rsync -avze --progress --inplace --rsh='ssh' src/ ubuntu@noehmer.no-ip.org:/home/ubuntu/tom/src


# Local variables:
# mode: Makefile
# End:
