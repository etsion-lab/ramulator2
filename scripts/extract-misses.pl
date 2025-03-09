#!/usr/bin/perl -w

use strict;

use Data::Dumper;

my $ninsts = 0;
my %cycles_per_core;
my $total_cycles = 0;

my @files = @ARGV;

foreach my $f (@files) {
    my @p = split(/\//, $f);
    my $name = $p[scalar(@p) - 2];

    open(my $fd, "<$f") or die "Error openning file $f: $!";

    my $llc_read_misses = 0;
    my $llc_write_misses = 0;
    my $cows_cache_misses = 0;

    while(<$fd>) {
        chomp;
        next if(/^\s*\#/);

        $llc_read_misses = $1 if(/^\s*llc_read_misses:\s+(\d+)\s*$/);
        $llc_write_misses = $1 if(/^\s*llc_write_misses:\s+(\d+)\s*$/);
        $cows_cache_misses = $1 if(/^\s*cows_cache_misses:\s+(\d+)\s*$/);
    }

    my $llc_misses = $llc_read_misses + $llc_write_misses;
    printf("%-30s: COWS/LLC misses=%.3f # (cows_misses: %d, llc_misses: %d (%d+%d))\n",
            $name, $cows_cache_misses/$llc_misses, $cows_cache_misses, $llc_misses, $llc_read_misses, $llc_write_misses);
}
