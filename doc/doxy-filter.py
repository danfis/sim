#!/usr/bin/env python
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
