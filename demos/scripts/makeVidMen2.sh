#!/bin/sh

#makes video using mencoder using two pass algorithm

#if video cannot be created - an error like 'aspect ratio error'
#of duplicated freames error occures,
#then: a) convert all png's to jpg
#      b) run 'convert pictures.png -depth 8 pictures8.png' on all png's


rm -f frameno.avi
mencoder "mf://*.png" -mf fps=20 -o test.avi -ovc lavc -lavcopts vcodec=mpeg4:vpass=1:vbitrate=2160000:vqmin=3
mencoder "mf://*.png" -mf fps=20 -o test.avi -ovc lavc -lavcopts vcodec=mpeg4:vpass=2:vbitrate=2160000:vqmin=3


avifix -i test.avi -F DIVX
rm divx2pass.log
