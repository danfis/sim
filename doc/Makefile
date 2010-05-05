-include ../Makefile.include

TARGETS = doxy

all: $(TARGETS)

doxy: Doxyfile
	rm -rf doxy
	$(MAKE) doc
	
doc:
	$(DOXYGEN)

.PHONY: all doc
