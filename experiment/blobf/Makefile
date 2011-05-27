CFLAGS += -I.
LDFLAGS = -lm -lrt

TARGETS = blobf
OBJS = blobf.o camera.o image.o finder.o

all: $(TARGETS)

pagesize: pagesize.c
	$(CC) $(CFLAGS) -o $@ $<
blobf: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c %.h
	$(CC) $(CFLAGS) $(DEFS) -c -o $@ $<
%.o: %.c
	$(CC) $(CFLAGS) $(DEFS) -c -o $@ $<

clean:
	rm -f $(OBJS)
	rm -f $(TARGETS)
	rm -f ccd/config.h
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;
	
check:
	$(MAKE) -C testsuites check
check-valgrind:
	$(MAKE) -C testsuites check-valgrind

help:
	@echo "Targets:"
	@echo "    all     - Build library"
	@echo "    install - Install library into system"
	@echo ""
	@echo "Options:"
	@echo "    CC - Path to C compiler"
	@echo "    CROSS_COMPILE - Prefix for gcc (I use bfin-linux-uclibc)"
	@echo "    M4 - Path to m4 macro processor"
	@echo ""
	@echo "    DEBUG 'yes'/'no'      - Turn on/off debugging (default: '$(DEBUG)')"
	@echo "    PROFIL 'yes'/'no'     - Compiles profiling info (default: '$(PROFIL)')"
	@echo "    NOWALL 'yes'/'no'     - Turns off -Wall gcc option (default: '$(NOWALL)')"
	@echo "    NOPEDANTIC 'yes'/'no' - Turns off -pedantic gcc option (default: '$(NOPEDANTIC)')"
	@echo "    USE_ARRAY 'yes'/'no'  - Turns on/off usage of arrays instead of arr_t (default: $(USE_ARRAY))"
	@echo ""
#	@echo "    PREFIX     - Prefix where library will be installed (default: /usr/local)"
#	@echo "    INCLUDEDIR - Directory where header files will be installed (PREFIX/INCLUDEDIR) (default: include)"
#	@echo "    LIBDIR     - Directory where library will be installed (PREFIX/LIBDIR) (default: lib)"
#	@echo ""


.PHONY: all clean check check-valgrind help
