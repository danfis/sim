-include ../Makefile.include

TARGETS = doxy sim-design.png

all: $(TARGETS)

doxy: Doxyfile
	$(MAKE) doc
	
sim-design.png: sim-design.dia
	$(DIA) -t svg -e sim-design.svg sim-design.dia
	$(CONVERT) sim-design.svg -resize 650 $@
	rm -f sim-design.svg

doc:
	rm -rf doxy
	$(DOXYGEN)
	./open-inheritance.py doxy/html/*.html

clean:
	rm -rf $(TARGETS)

.PHONY: all doc
