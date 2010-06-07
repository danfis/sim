###
# sim
# ---------------------------------
# Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
#
#  This file is part of sim.
#
#  sim is free software; you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as
#  published by the Free Software Foundation; either version 3 of
#  the License, or (at your option) any later version.
#
#  sim is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

-include ../Makefile.local
-include ../Makefile.include

TARGETS = doxy/html sim-design.png

all: $(TARGETS)

doxy/html: Doxyfile
	$(MAKE) doc
	
sim-design.png: sim-design.dia
	$(DIA) -t svg -e sim-design.svg sim-design.dia
	$(CONVERT) sim-design.svg -resize 650 $@
	rm -f sim-design.svg

doc:
	rm -rf doxy/html
	$(DOXYGEN)
	./open-inheritance.py doxy/html/*.html

clean:
	rm -rf $(TARGETS)

.PHONY: all doc
