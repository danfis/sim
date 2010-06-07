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


#converts 'raw' format to c/c++ header file
#with definition of array suitable for loading them as mesh into ODE library

if (@ARGV < 3) {
	print STDERR "usage: $0 <inFile.raw> <outFile.h> <variableName\n";
	print STDERR "<inFile.raw>   file with mesh in RAW format\n";
	print STDERR "<outFile.h>    output c/c++ header file \n";
	print STDERR "<variableName  name of variable definig the esh in .h file\n";
	exit;
}

my ($infile,$outfile, $varName) = @ARGV;

open(my $fi,"<$infile") or die "$! $infile!\n";
my @data = <$fi>;
close $fi;

my $fn = uc($outfile);
$fn =~ s/\./_/g;
$fn =~ s/\///g;

open(my $fo, ">$outfile") or die "$! $outfile!\n";
	
print $fo "#ifndef $fn\n#define $fn\n\n";
print $fo "#include <sim/math.hpp>\n";
print $fo "static sim::Vec3 ${varName}_verts[] = {\n";

for(my $i=0;$i<@data;$i++) {
	my @a = split(/\s+/,$data[$i]);
	for(my $j=0;$j<3;$j++) {
		print $fo "sim::Vec3(".$a[3*$j].",".$a[3*$j+1].",".$a[$j*3+2].")";
		if ($j < 2) {
			print $fo ",\n";
		}
	}
	if ($i < @data-1) {
		print $fo ",\n";
	}
}
print $fo "};\n\n";

print $fo "const size_t ${varName}_verts_len = sizeof(${varName}_verts) / sizeof(sim::Vec3);\n\n";

print $fo "unsigned int ${varName}_ids[] = {\n";
my $idx = 0;
for(my $i=0;$i<@data;$i++) {
	foreach my $q (0..2) {
		print $fo $idx++;
		if ($q < 2) {
			print $fo ",";
		}	
	}
	if ($i < @data-1) {
		print $fo ",\n";
	}
}
print $fo "};\n\n";

print $fo "const size_t ${varName}_ids_len = sizeof(${varName}_ids)/sizeof(unsigned int);\n";

print $fo "#endif\n";
close $fo;
