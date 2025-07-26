#!/usr/bin/perl -w

use strict;
use Getopt::Long;
use experimental 'smartmatch';

use File::Copy; # Include for moving files
use Data::Dumper;

our $same_asid = 1; # <-- Declare here, before any subroutines

my $BENCHDIR="/scratch/elior.k/cloudsuite-traces";
my $BINARY="/scratch/elior.k/ramulator2/build/ramulator2";

my $RESDIR="/scratch/elior.k/ramulator2/res";

my %BENCHMARKS = (
    "data-analytics-core" =>    { "cores" => [ 1, 8 ],  "half" => 0 },
    "data-caching-core" =>      { "cores" => [ 1, 8 ],  "half" => 1 },
    "data-serving-core" =>      { "cores" => [ 1, 8 ],  "half" => 1 },
    "graph-analytics-core" =>   { "cores" => [ 1, 8 ],  "half" => 0 },
    "in-memory-analytics-core" => { "cores" => [ 1, 8 ],    "half" => 0 },
    "media-streaming-core" =>   { "cores" => [ 1, 8 ],  "half" => 1 },
    "web-search-core" =>        { "cores" => [ 1 ],     "half" => 1 },
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
    (my $name, my $cores, my $d, my $asid_mode) = @_;

    my $start_core = 0;
    $start_core = $cores if($d->{"half"});

    my @ret;
    for my $i ($start_core .. ($start_core + $cores - 1)) {
        my $trace_path = "$BENCHDIR/$name-$cores/trace.cpu$i";
        my $asid_str;
        if ($same_asid) {
            $asid_str = ":0";
        } else {
            $asid_str = ":$i";
        }
        if (-e $trace_path) {
            push(@ret, "  - $trace_path$asid_str");
        } else {
            print "Warning: Trace file not found: $trace_path\n";
        }
    }

    @ret;
}

sub build_trace_list {
    my @benchs = @_;
    my @ret;

    for my $b (@benchs) {
        (my $bname, my $c, my $d) = find_bench($b);
        push(@ret, bench_to_trace($bname, $c, $d, $same_asid));
    }

    print "Generated trace list:\n";
    print Dumper(\@ret); # Debugging output
    @ret;
}

sub gen_config($$$) {
    (my $in, my $out, my $map) = @_;

    open(my $fin, '<', $in) or die "Could not open input config file $in: $!";
    open(my $fout, '>', $out) or die "Could not open output config file $out: $!";

    while (<$fin>) {
        chomp;
        next if (/^\s*\#/); # Skip comments

        my $line = $_;

        # Handle the TRACES macro explicitly for the traces list
        if ($line =~ /^\s*traces:\s*$/) {
            print $fout "  traces:\n";
            if (defined($map->{"TRACES"})) {
                print "Writing traces to config:\n"; # Debugging output
                foreach my $trace (@{$map->{"TRACES"}}) {
                    print "  $trace\n"; # Debugging output
                    print $fout "  $trace\n"; # Ensure proper indentation
                }
            } else {
                die "Error: TRACES macro is not defined!";
            }
        }
        # Handle other macros dynamically
        elsif ($line =~ /^(\s*)([A-Z_]+[0-9]*)\s*$/) {
            my $s = $1;
            my $macro = $2;

            die "Error: undefined macro \"$macro\"" if (!defined($map->{$macro}));
            my @l = @{$map->{$macro}};
            foreach (@l) {
                print $fout "$s" . $_ . "\n";
            }
        }
        # Copy other lines as-is
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

# Add option for address space mode
GetOptions (
    "bench=s" => \@benchs,
    "name=s" => \$test_name,
    "dir=s" => \$rundir_prefix,
    "config=s" => \$confile,
    "help" => \$help,
    "same-asid!" => \$same_asid, # --same-asid to force same, --no-same-asid for split
) or die("Error in command line arguments\n");

if($help) {
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
$rundir = "$RESDIR/$rundir_prefix/$test_name" if(defined($rundir_prefix));
system("mkdir -p $rundir");

# create config file
my $outconf = "$rundir/config";
gen_config($confile, $outconf, \%macros);

# # Add this block to print the contents of the trace directory
# print "Checking contents of trace directory: $BENCHDIR\n";
# if (opendir(my $dh, $BENCHDIR)) {
#     my @files = readdir($dh);
#     closedir($dh);
#     print "Files in $BENCHDIR:\n";
#     foreach my $file (@files) {
#         print "  $file\n";
#     }
# } else {
#     print "Error: Could not open trace directory $BENCHDIR: $!\n";
# }

# Run the simulation command
my $cmd="/usr/bin/time -v $BINARY -f $outconf > out 2>&1";

print "Changing dir to: $rundir\n";
chdir $rundir;

print "Running command: $cmd\n";
system($cmd); # This will block until the command finishes

# Rename and move the CSV file
my $csv_file = "/scratch/elior.k/ramulator2/res/csv_outputs/row_histograms.csv";
if (-e $csv_file) {
    my $renamed_csv = "/scratch/elior.k/ramulator2/res/csv_outputs/${test_name}_row_metrics.csv";
    my $destination = "$rundir/${test_name}_row_metrics.csv";

    # Rename the CSV file locally in csv-outputs
    print "Renaming $csv_file to $renamed_csv\n";
    move($csv_file, $renamed_csv) or die "Error: Could not rename $csv_file to $renamed_csv: $!";

    # Move the renamed file to the output folder
    print "Moving $renamed_csv to $destination\n";
    move($renamed_csv, $destination) or die "Error: Could not move $renamed_csv to $destination: $!";
} else {
    print "Warning: CSV file $csv_file not found. Skipping rename and move.\n";
}

# Rename and move the CSV file generated by the controller!
my $csv_file = "/scratch/elior.k/ramulator2/res/csv_outputs/row_metrics_generic_controller.csv";
if (-e $csv_file) {
    my $renamed_csv = "/scratch/elior.k/ramulator2/res/csv_outputs/${test_name}_row_metrics_cntrl.csv";
    my $destination = "$rundir/${test_name}_row_metrics_cntrl.csv";

    # Rename the CSV file locally in csv-outputs
    print "Renaming $csv_file to $renamed_csv\n";
    move($csv_file, $renamed_csv) or die "Error: Could not rename $csv_file to $renamed_csv: $!";

    # Move the renamed file to the output folder
    print "Moving $renamed_csv to $destination\n";
    move($renamed_csv, $destination) or die "Error: Could not move $renamed_csv to $destination: $!";
} else {
    print "Warning: CSV file $csv_file not found. Skipping rename and move.\n";
}
