#!/usr/bin/env perl

#converts 'raw' format to c/c++ header file
#with definition of array suitable for loading them as mesh into ODE library

if (@ARGV < 2) {
	print STDERR "usage: $0 <inFile.raw> <outFile.h>\n";
	print STDERR "<inFile.raw>   file with mesh in RAW format\n";
	print STDERR "<outFile.h>    output c/c++ header file \n";
	exit;
}

my ($infile,$outfile) = @ARGV;

open(my $fi,"<$infile") or die "$! $infile!\n";
my @data = <$fi>;
close $fi;

my $fn = uc($outfile);
$fn =~ s/\./_/g;
$fn =~ s/\///g;

open(my $fo, ">$outfile") or die "$! $outfile!\n";
	
print $fo "#ifndef $fn\n#define $fn\n\n";

print $fo "Vec3 verts[] = {\n";

for(my $i=0;$i<@data;$i++) {
	my @a = split(/\s+/,$data[$i]);
	for(my $j=0;$j<3;$j++) {
		print $fo "Vec4(".$a[3*$j].",".$a[3*$j+1].",".$a[$j*3+2].")";
		if ($j < 2) {
			print $fo ",\n";
		}
	}
	if ($i < @data-1) {
		print $fo ",\n";
	}
}
print $fo "};\n\n";

print $fo "unsigned int ind[] = {\n";
for(my $i=0;$i<@data;$i++) {
	print $fo " 0,1,2,\n3,4,5\n6,7,8";
	if ($i < @data-1) {
		print $fo ",\n";
	}
}
print $fo "};\n\n";


print $fo "#endif\n";
close $fo;
