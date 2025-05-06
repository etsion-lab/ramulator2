#!/bin/bash
# Single Core
perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name fix_ovf_collect_metrics_SC_data-analytics-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench data-analytics-core-1
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_SC_data-caching-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench data-caching-core-1
perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name fix_ovf_collect_metrics_SC_data-serving-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench data-serving-core-1
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_SC_graph-analytics-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench graph-analytics-core-1
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_SC_in-memory-analytics-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench in-memory-analytics-core-1
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_SC_media-streaming-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench media-streaming-core-1
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_SC_web-search-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench web-search-core-1
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_SC_web-serving-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench web-serving-core-1

# Plot histograms for Single Core
echo "Plotting histograms for Single Core tests..."
python3 scripts/plot_hists.py res/fix_ovf_collect_metrics_SC_*/*.csv

# Multi Core
perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name fix_ovf_collect_metrics_MC_data-analytics-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench data-analytics-core-8
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_MC_data-caching-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench data-caching-core-8
perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name fix_ovf_collect_metrics_MC_data-serving-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench data-serving-core-8
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_MC_graph-analytics-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench graph-analytics-core-8
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_MC_in-memory-analytics-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench in-memory-analytics-core-8
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_MC_media-streaming-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench media-streaming-core-8
# perl /scratch/elior.k/ramulator2/scripts/run_ek.pl --name collect_metrics_MC_web-serving-core --config /scratch/elior.k/ramulator2/example_config_ek.yaml --bench web-serving-core-8

# Plot histograms for Multi Core
echo "Plotting histograms for Multi Core tests..."
python3 scripts/plot_hists.py res/fix_ovf_collect_metrics_MC_*/*.csv

# data-analytics-core
# data-caching-core
# data-serving-core
# graph-analytics-core
# in-memory-analytics-core
# media-streaming-core
# web-search-core
# web-serving-core