#!/usr/bin/perl -w

use strict;

use Data::Dumper;

my @files = @ARGV;

foreach my $f (@files) {
    my %cycles_per_core = ();
    my $total_cycles = 0;

    my %insts_per_core = ();
    my $total_insts = 0;


    my @p = split(/\//, $f);
    my $name = $p[scalar(@p) - 2];

    open(my $fd, "<$f") or die "Error openning file $f: $!";

    while(<$fd>) {
        chomp;
        next if(/^\s*\#/);

        if(/^\s*cycles_recorded_post_warmup_core_(\d+):\s+(\d+)\s*$/) {
            my $core = int($1);
            my $cycles = int($2);
            $cycles_per_core{$core} = $cycles;
            $total_cycles += $cycles;
        }
        elsif(/^\s*insts_retired_post_warmup_core_(\d+):\s+(\d+)\s*$/) {
            my $core = int($1);
            my $ninsts = int($2);
            $insts_per_core{$core} = $ninsts;
            $total_insts += $ninsts;
        }
    }

    my $t=0;
    my $ncores = scalar(keys %cycles_per_core);
    for(my $c=0; $c<$ncores; $c++) {
        my $cy = $cycles_per_core{$c};
        my $insts = $cycles_per_core{$c};
#        printf("IPC[%3d]=%4.2f\n", $c, ($insts/$cy));
    }
    if($total_cycles == 0) {
        print("IPC: Missing cycles in file $f\n");
        next;
    }
    printf("%s\n", $f);
    printf("%-30s: IPC[Total]=%.2f # (Total insts: %d, Total cycles: %d)\n",
            $name, ($total_insts/$total_cycles), ($total_insts), $total_cycles);
}
