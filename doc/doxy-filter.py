#!/usr/bin/env python
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

###
# Filter for doxygen.
# Transforms comments to brief description will be everything from
# beggining to first empty line. Example:
#   /**
#    * Brief description over
#    * several lines.
#    *
#    * And this is the rest
#    */
#
#    transforms into:
#
#    /**
#     * \brief Brief description over
#     * several lines.
#     *
#     * And this is the rest
#     */
#
# Dont forget to disable all autobrief options in Doxyfile configuration.
#

import sys
import re

pat_start = re.compile(r'^.*/\*\*$')
pat_end = re.compile(r'^.*\*/$')
pat_middle = re.compile(r'^(.*\* )(.*)$')
pat_middle_space = re.compile(r'^.*\*\s*$')


# if true previous line was start of comment block
found_start = False

# open file
fin = open(sys.argv[1])

# iterate over all lines
for line in fin:
    if found_start:
        match = pat_middle.match(line)
        if match:
            begining = match.group(1)
            text = match.group(2)
            if not text.startswith(r'\brief'):
                line = begining
                line += r'\brief '
                line += text
                line += '\n'
        found_start = False

    else:
        match = pat_start.match(line)
        if match:
            found_start = True

    print line,

# close file
fin.close()
