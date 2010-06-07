#!/bin/sh
###
# sim
# ---------------------------------
# Copyright (c)2010 Vojta Vonasek <vonasek@labe.felk.cvut.cz>
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
