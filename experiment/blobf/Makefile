-include ../../Makefile.include

CXXFLAGS += -I. -I../../src
LDFLAGS = -lm -lrt

TARGETS = libblobf.a
OBJS = finder.o

all: $(TARGETS)

libblobf.a: $(OBJS)
	ar cr $@ $(OBJS)
	ranlib $@

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) $(DEFS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(DEFS) -c -o $@ $<

clean:
	rm -f $(OBJS)
	rm -f $(TARGETS)
	rm -f ccd/config.h
	if [ -d testsuites ]; then $(MAKE) -C testsuites clean; fi;
	


.PHONY: all clean check check-valgrind help
