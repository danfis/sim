#!/usr/bin/env perl
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


#process rpovray files in ./ directory

my $A = 0.001;

if (@ARGV < 1) {
	print STDERR "usage: <type>\n";
	print STDERR "type   0 800x600 no antialias\n";
	print STDERR "       1 800x600 with antialias: $A\n";
	print STDERR "       2 1024x768 no antialias\n";
	print STDERR "       3 1024x768 with antialias: $A\n";
	print STDERR "       4 600x450 no antialias\n";
	print STDERR "       5 600x450 with antialias: $A\n";
	exit;
}
use Fcntl qw/ :flock /;


my ($type) = @ARGV;

my $c = "-W800 -H600 -D";
if ($type == 1) {
	$c = "-W800 -H600 -D +A$A";
} elsif ($type == 2) {
	$c = "-W1024 -H768 -D";
} elsif ($type == 3) {
	$c = "-W1024 -H768 -D +A$A";
} elsif ($type == 4) {
	$c = "-W600 -H450 -D ";
} elsif ($type == 5) {
	$c = "-W600 -H450 -D +A$A";
}

foreach(<*.pov>) {
	my $f = $_;
	$f =~ s/\.pov$/\.png/;
	if ( ! (-f $f)) {
		my $lockf = "$f.lock";
		open($fl,">$lockf") or die "$! $lockf\n";
		if (flock($fl,LOCK_EX | LOCK_NB)) {
			system "povray $_ $c";	
			close $fl;
			unlink($lockf);	
		}
		close $fl;
	}
}

