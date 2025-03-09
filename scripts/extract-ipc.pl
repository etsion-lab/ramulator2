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

    while(<$fd>) {
        chomp;
        next if(/^\s*\#/);

        if(/^\s*cycles_recorded_core_(\d+):\s+(\d+)\s*$/) {
            my $core = int($1);
            my $cycles = int($2);
            $cycles_per_core{$core} = $cycles;
            $total_cycles += $cycles;
        }
        elsif(/num_expected_insts:\s+(\d+)\s*$/) {
            $ninsts = $1;
        }
    }

    my $t=0;
    my $ncores = scalar(keys %cycles_per_core);
    for(my $c=0; $c<$ncores; $c++) {
        my $cy = $cycles_per_core{$c};
#        printf("IPC[%3d]=%4.2f\n", $c, ($ninsts/$cy));
    }
    printf("%-30s: IPC[Total]=%.2f # (Total insts: %d, Total cycles: %d)\n",
            $name, (($ninsts*$ncores)/$total_cycles), ($ninsts*$ncores), $total_cycles);
}
