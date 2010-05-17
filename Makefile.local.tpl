###
# Template Makefile variable definition file.
#
# Makefile.local file located on top of source tree is included as first to
# all Makefiles. This file isn't tracked in git repository (is named in
# .gitignore file) so you can modify it on your local machine to have
# correctly set up compile chain and you can git push/pull at same time
# without any problems.
#
# It should be enough to set up here some variables and all Makefiles
# should accept that - if any variable behaves naughty you should spank it
# and report the problem to responsible developer as bug (currently it is
# Dan Fiser <danfis [at] danfis [dot] cz>).
#
# If you don't know how to set up which variable follow these steps
# (FreeBSD users can skip to 5): 
#
#   1) Run 'make showvars' - this prints out all defined variables whichs
#      names should be self-explanory.
#   2) Run 'make check-dep' - this tries to find all dependencies and shows
#      the result.
#   3) Read Makefile.include and eventually some other Makefile to
#      understand the whole picture.
#   4) If everything failed be sure you have tried previous steps and some
#      of your own imagination, your collegues, friends, mummy etc.
#      Then, when nothing helped, try to contact responsible developer to
#      help.
#   5) If you have any lousy operating system, such as FreeBSD, try to burn
#      whole computer and go to mountains and become a monk. It really
#      helps, a lot, really.
#

# Turn debugging off
# DEBUG = no

# Don't use -pedantic flag in gcc command
# NOPEDANTIC = yes

