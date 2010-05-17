-include Makefile.local
-include Makefile.include

TARGETS = src demos

all: $(TARGETS)

src:
	$(MAKE) -C src

demos: src
	$(MAKE) -C demos

clean:
	$(MAKE) -C src clean
	$(MAKE) -C demos clean

.PHONY: all clean src demos
