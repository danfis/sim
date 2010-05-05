#!/usr/bin/env python

###
# This script takes html files generated from doxygen as arguments.
# All these files are modified to show inheritance diagram when page is
# opened.
#

import re
import sys
import os


pat_div = re.compile(r'^.*<div .*>.*$')
pat_onload = re.compile(r'^window.onload = initDynSections;.*$')


script = '''
<script type="text/javascript"><!--
function __initDynSections() {
    initDynSections();

    var __divs = document.getElementsByTagName('div');
    var __sectionCounter = 1;
    var __done = 0;
    for(var i = 0; i < __divs.length - 1 && !__done; i++){
        if(__divs[i].className == 'dynheader' && __divs[i+1].className == 'dynsection'){
            var __ch = __divs[i + 1].children;
            for (var j = 0; j < __ch.length - 1; j++){
                if (__ch[j].name && __ch[j].name.indexOf('inherit') != -1){
                    changeDisplayState.apply(__divs[i]);
                    __done = 1;
                    break;
                }
            }
        }
    }
}

window.onload = __initDynSections;
--></script>
'''


def replace(fnin):
    os.system('cp {0} .tmp'.format(fnin))

    fout = open(fnin, 'w')
    fin = open('.tmp', 'r')

    done_div = False
    done_onload = False
    for line in fin:
        if not done_div:
            match = pat_div.match(line)
            if match:
                fout.write(script)
                done_div = True

        if not done_onload:
            match = pat_onload.match(line)
            if match:
                line = ''
                done_onload = True

        fout.write(line)

    fin.close()
    fout.close()

    os.system('rm -f .tmp')

print >>sys.stderr, ""
print >>sys.stderr, "Modifying files..."
for f in sys.argv[1:]:
    print >>sys.stderr, "  -->", f
    replace(f)
