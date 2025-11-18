#!/usr/bin/perl -w

use strict;
use Getopt::Long;
use experimental 'smartmatch';
use Data::Dumper;

my $BENCHDIR="/scratch/yetsion/cows/cloudsuite-traces/zero-copy-insts/";
my $BINARY="/scratch/yetsion/cows/tools/ramulator2/build/ramulator2";

my $RESDIR="/scratch/yetsion/cows/tools/ramulator2/res";

my %BENCHMARKS = (
    "data-analytics-core" =>    { "cores" => [ 1, 8 ],  "half" => 0 },
    "data-caching-core" =>      { "cores" => [ 1, 8 ],  "half" => 1 },
    "data-serving-core" =>      { "cores" => [ 1, 8 ],  "half" => 1 },
    "graph-analytics-core" =>   { "cores" => [ 1, 8 ],  "half" => 0 },
    "in-memory-analytics-core" => { "cores" => [ 1, 8 ],    "half" => 0 },
    "media-streaming-core" =>   { "cores" => [ 1, 8 ],  "half" => 1 },
    "web-search-core" =>        { "cores" => [ 1, 8 ],     "half" => 1 },
    "web-serving-core" =>       { "cores" => [ 1, 8 ],  "half" => 1 },
);

#
# Helpers
#
sub find_bench($) {
    my $b = shift;

    if($b !~ /^(.+)-(\d)$/) {
        die "Cannot parse benchmark name \"$b\"";
    }
    my $bnick = $1;
    my $cores = $2;

    my $bname = undef;
    foreach my $bn (keys(%BENCHMARKS)) {
        if(index($bn, $bnick) == 0) {
            if(defined($bname)) {
                die "benchmark nickname \"$bnick\" is not unique";
            }
            if(!($cores ~~ $BENCHMARKS{$bn}{"cores"})) {
                die "benchmark \"$bn\" does not support $cores cores"
            }
            $bname = $bn;
        }
    }

    return ($bname, $cores, $BENCHMARKS{$bname});
}

sub bench_to_trace($$$$) {
    (my $name, my $cores, my $d, my $asid) = @_;

    my $start_core = 0;
    $start_core = $cores if($d->{"half"});

    my @ret;
    for ($start_core .. ($start_core + $cores - 1)) {
        my $c = $_;

        push(@ret, "- $BENCHDIR/$name-$cores/trace.cpu$c:$asid");
    }

    @ret;
}

sub build_trace_list {
    my @benchs = @_;
    my @ret;

    my $asid = 0;
    for my $b (@benchs) {
        (my $bname, my $c, my $d) = find_bench($b);

        push(@ret, bench_to_trace($bname, $c, $d, $asid));
        $asid++;
    }

    @ret;
}

sub gen_config($$$) {
    (my $in, my $out, my $map) = @_;

    open(my $fin, '<', $in) or die "Could not open input config file $in: $!";
    open(my $fout, '>', $out) or die "Could not open input config file $out: $!";

    while(<$fin>) {
        chomp;
        next if(/^\s*\#/); # skip comments

        my $line = $_;

        # find macros
        if($line =~ /^(\s*)([A-Z_]+[0-9]*)\s*$/) {
            my $s = $1;
            my $macro = $2;

            die "Error: undefined macro \"$macro\"" if(!defined($map->{$macro}));
            my @l = @{$map->{$macro}};
            foreach (@l) {
                print $fout "$s" . $_ . "\n";
            }
        }
        else {
            print $fout "$line\n";
        }
    }

    close($fin);
    close($fout);
}

sub usage($) {
    printf("%s: <--name run-name> <--config config-file> [-d rundir-prefix] <--bench bench1> [--bench bench2...]\n", $0);
}

#
# Main
#
my @benchs;
my $test_name;
my $confile;
my $rundir_prefix;
my $help;

my $argc = scalar(@ARGV);

GetOptions ("bench=s" => \@benchs,
            "name=s" => \$test_name,
            "dir=s" => \$rundir_prefix,
            "config=s" => \$confile,
            "help" => \$help,
            "h" => \$help)
    or die("Error in command line arguments\n");

if($argc==0 or $help) {
    usage($0);
    exit(0);
}
die "Error: no benchmarks given" if(scalar(@benchs) == 0);
die "Error: no test name specified" if(!defined($test_name));
die "Error: no config file specified" if(!defined($confile));

my %macros;
@{$macros{"TRACES"}} = build_trace_list(@benchs);

# create rundir
my $rundir = "$RESDIR/$test_name";
if(defined($rundir_prefix)) {
    $rundir = "$RESDIR/$rundir_prefix/$test_name";
}
system("mkdir -p $rundir");
system("cp $confile $rundir/"); # save template config

# create config file
my $outconf = "$rundir/config";
gen_config($confile, $outconf, \%macros);

#my $cmd="/usr/bin/time -v $BINARY -f $outconf 2>&1 > out";
my $cmd="/usr/bin/time -v $BINARY -f $outconf > out 2>&1";

print "Changing dir to: $rundir\n";
chdir $rundir;

print "Running command: $cmd in dir $rundir\n";
system($cmd);
