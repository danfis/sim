-include ../Makefile.include

TARGETS = doxy

all: $(TARGETS)

doxy: Doxyfile
	$(MAKE) doc
	
doc:
	rm -rf doxy
	$(DOXYGEN)

clean:
	rm -rf doxy

.PHONY: all doc
