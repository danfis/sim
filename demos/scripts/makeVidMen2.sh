#!/bin/sh

#makes video using mencoder using two pass algorithm from *.png files in current directory

#if the video cannot be created due to 'aspect ratio error' or 'duplicated fremes error',
#then: a) convert all png's to jpg
#      b) run 'convert pictures.png -depth 8 pictures8.png' on all png's


rm -f frameno.avi
mencoder "mf://*.png" -mf fps=20 -o test.avi -ovc lavc -lavcopts vcodec=mpeg4:vpass=1:vbitrate=2160000:vqmin=3
mencoder "mf://*.png" -mf fps=20 -o test.avi -ovc lavc -lavcopts vcodec=mpeg4:vpass=2:vbitrate=2160000:vqmin=3

#this command set proper codec to result .avi file - otherwise it is possible that the video
#cannot be played on Windows
#the program 'avifix' is part of transcode (http://www.transcoding.org/)

avifix -i test.avi -F DIVX
rm divx2pass.log
