-include ../Makefile.include

TARGETS = doxy

all: $(TARGETS)

doxy: Doxyfile
	$(MAKE) doc
	
doc:
	rm -rf doxy
	$(DOXYGEN)
	./open-inheritance.py doxy/html/*.html

clean:
	rm -rf doxy

.PHONY: all doc
