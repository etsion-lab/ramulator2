This readme describes the work that has been done on the working copy of ramulator2 as groundwork of potential research in the idea of optimizing the DRAM refresh.
The work done consists of these main parts:
1. metry collection of the following metrics related to the dram behaviour:
  1. average row open duration - how much time (in cpu cycles) does the dram row remain open.
  2. row open count
  3. row average re-open interval - what is the average time that it takes for a row to be re-opened after closed.
  4. Total refresh AR commands - the amount of per-row refresh commands.
  5. Full refresh cycles completed
  6. Average number of refreshes between re-openes of a row - meaning how many refrehses have opened the row when in fact it was idle/data was not accessed.

The metrics are exported to a csv file, and are printed for each row for the selected DRAM channels/ranks/bg. 
The metry collection was initially implemented on the frontend part of the ramulator simulator by adding logging logic to a local copy of DDR5.cpp file (DDR5_EK.cpp),
which is a file implementing the frontend of the memory implementation with access to to handles of all incoming DRAM commands.
Later on, after suspecting that not all memory commands are passed through to the frontend, the metry collection was then implemented as updates in the generic_dram_controller.cpp backend file, in order to account for row conflicts and misses that cause row closes which are not logged as a standalone command.

2. run scripts, benchmarks and parsers.

-----------------------------------------------------------------------------------------------------------------

# run_ek.pl

This script is a **benchmark automation and results management tool** for Ramulator2 DRAM simulator experiments. It streamlines the process of running benchmarks, generating configuration files, and organizing output metrics for later analysis.

---

## Features

- **Flexible Benchmark Selection:**  
  Supports running any combination of CloudSuite benchmarks with single or multi-core configurations.

- **Address Space Control:**  
  Allows running multi-core traces in either the same or different address spaces using the `--same-asid` or `--no-same-asid` flags.

- **Automatic Config Generation:**  
  Dynamically generates a YAML configuration file for each run, inserting the correct trace file paths and macros.

- **Output Directory Management:**  
  Creates a dedicated results directory for each run, named after the test.

- **Simulation Execution:**  
  Runs the Ramulator2 binary with the generated configuration, capturing output and resource usage.

- **Results Renaming and Organization:**  
  After simulation, automatically renames and moves the generated CSV output files (row metrics and controller metrics) into the run's results directory, using the test name as a prefix for easy identification.

- **Error Handling and Debugging:**  
  Prints warnings for missing trace files or CSV outputs, and includes debug output for trace list generation.

---

## Usage

```sh
perl run_ek.pl --name <run-name> --config <config-file> [--dir <rundir-prefix>] --bench <bench1> [--bench <bench2> ...] [--same-asid|--no-same-asid]
```

- `--name` : Name for this run (used for directory and output file naming)
- `--config` : Path to the base YAML config file
- `--dir` : (Optional) Prefix for the results directory
- `--bench` : Benchmark(s) to run (e.g., `data-analytics-core-1`)
- `--same-asid` : (Default) All traces use the same address space
- `--no-same-asid` : Each trace uses a different address space

**Example:**
```sh
perl run_ek.pl --name test_run --config example_config_ek.yaml --bench data-analytics-core-1 --bench web-serving-core-8 --no-same-asid
```

---

## Output

- **Results Directory:**  
  All outputs for a run are placed in `res/<run-name>` (or `res/<rundir-prefix>/<run-name>` if a prefix is given).

- **CSV Files:**  
  - `<run-name>_row_metrics.csv` (row-level metrics from DRAM frontend)
  - `<run-name>_row_metrics_cntrl.csv` (row-level metrics from the controller backend, if available)

- **Simulation Output:**  
  - `out` (stdout and stderr from the Ramulator2 run)

---

## Requirements

- Perl 5
- Ramulator2 binary and CloudSuite traces available at the specified paths
- Proper permissions to create directories and move files

---

## Customization

- **Benchmarks:**  
  Edit the `%BENCHMARKS` hash to add or modify supported benchmarks and core counts.

- **Paths:**  
  Update `$BENCHDIR`, `$BINARY`, and `$RESDIR` as needed for your environment.

-----------------------------------------------------------------------------------------------------------------

# plot_hists.py

This script generates visualizations and summary statistics from one or more DRAM row-metrics CSV files, typically produced by Ramulator2 or similar memory simulators.

---

## Usage

```sh
python plot_hists.py <csv_file1> [<csv_file2> ...]
```

- You can provide one or multiple CSV files as input.
- For each input file, the script generates a PDF report with plots and tables.
- If multiple CSVs are provided, a merged PDF report is also generated for cross-file comparison.

---

## What the Script Does

### For Each Input CSV File

- **Reads the CSV** containing per-row DRAM metrics (such as OpenCount, AvgOpenDuration, etc.).
- **Generates a PDF report** with the following visualizations and summaries:

  - **Bank Row Access Summary:**  
    Table showing the number of unique rows accessed per bank (Channel, Rank, BankGroup, Bank).

  - **Bank Access Heatmap:**  
    Bar plot of total row opens per bank, visualizing which banks are most frequently accessed.

  - **Row Access Distribution:**  
    For each bank, a histogram and CDF showing which rows were accessed and how frequently, including the "hot row" range (interquartile range).

  - **Row Popularity:**  
    For each bank, a histogram and CDF of the OpenCount metric, with mean and median annotations, showing how many times each row was opened.

  - **Metric Histograms:**  
    For each of the following metrics:
    - `AvgOpenDuration` (average time a row stays open)
    - `OpenCount` (number of times a row was opened)
    - `AvgReopenInterval` (average interval between row opens)
    - `FullRefreshCycles` (number of refresh cycles a row experienced)
    - `AvgRefreshesBetweenReopens` (average number of refreshes between row opens)
    The script plots both a histogram and a cumulative distribution (CDF), with mean and median lines.

### When Multiple CSV Files Are Provided

- **Merges the data** on shared keys (Channel, Rank, BankGroup, Bank, Row).
- **Generates a merged PDF report** in `res/merged/`, containing:
  - Combined bank row access summary and heatmap.
  - Merged row access distributions and popularity plots.
  - Merged metric histograms for cross-run comparison.

---

## Output

- For each input CSV:  
  - A PDF report named `<csv_file_prefix>_plots.pdf` in the same directory as the CSV.
- For merged output (if multiple CSVs):  
  - A merged PDF report named `merged_plots_<timestamp>.pdf` in `res/merged/`.

---

## Metrics Visualized

- **AvgOpenDuration:** Average time (in cycles) each row stays open.
- **OpenCount:** Number of times each row was opened.
- **AvgReopenInterval:** Average interval (in cycles) between consecutive opens of the same row.
- **FullRefreshCycles:** Number of refresh cycles experienced by each row.
- **AvgRefreshesBetweenReopens:** Average number of refreshes between consecutive opens of the same row.

---

## Additional Visualizations

- **Bank Access Heatmap:**  
  Visualizes the total number of row opens per bank, helping identify hotspots in the memory system.

- **Bank Row Access Summary:**  
  Tabulates the number of unique rows accessed in each bank, providing a quick overview of memory utilization.

- **Row Popularity:**  
  Shows the distribution of how often each row is opened, highlighting skewed or uniform access patterns.

- **Row Access Distribution:**  
  Illustrates which rows within each bank are accessed most frequently, including cumulative distributions and hot row ranges.

---

## Requirements

- Python 3.x
- `numpy`, `pandas`, `matplotlib`

Install dependencies with:
```sh
pip install numpy pandas matplotlib
```

---

## Notes

- The script expects the CSVs to have columns: Channel, Rank, BankGroup, Bank, Row, and the above metrics.
- All plots and tables are saved in PDF format for easy sharing and publication.
- Apart from the row access distribution, the graphs do not display un-accessed rows to make it more visually comfortable.

---

-----------------------------------------------------------------------------------------------------------------

# DRAM Log Parser (`parse_open_close_rows.py`) - (NOTE: for when the debug prints are enabled in the DDR%_EK file (commented out))

This script parses DRAM controller debug logs to extract and analyze row open/close events, active buffer states, and debug events (HIT, MISS, CONFLICT). It is designed to help users understand DRAM row activity and diagnose memory access patterns in Ramulator2 or similar simulators.

---

## Features

- **Row Open/Close Tracking:**  
  Extracts all row open and close events, building a timeline for each row.

- **Active Buffer Snapshots:**  
  Parses and displays the state of the active buffer at various cycles, including all outstanding requests.

- **Debug Event Extraction:**  
  Captures and annotates debug events such as HIT, MISS, and CONFLICT, including details about conflicting rows.

- **Flexible Filtering:**  
  Allows filtering by channel, rank, bank group, and bank to focus on specific DRAM regions.

- **Summary and Chronological Output:**  
  - Lists all rows that remain open at the end of the run.
  - Shows the detailed open/close sequence for each row.
  - Provides a chronological log of all parsed events.

- **Output Options:**  
  Results can be printed to the console or written to a specified output file in Markdown table format.

---

## Usage

```sh
python parse_open_close_rows.py <log_file_path> [output_file_path] [--channel <id>] [--rank <id>] [--bg <id>] [--bank <id>]
```

- `<log_file_path>`: Path to the DRAM log file to parse (required).
- `[output_file_path]`: Optional path to write the output (Markdown format). If omitted, prints to console.
- `--channel <id>`: Filter events by channel ID.
- `--rank <id>`: Filter events by rank ID.
- `--bg <id>`: Filter events by bank group ID.
- `--bank <id>`: Filter events by bank ID.

**Example:**
```sh
python parse_open_close_rows.py dram_debug.log parsed_output.txt --channel 0 --rank 1
```

---

## Output Sections

1. **Rows Remaining Open at the End of the Run:**  
   Table listing all rows that were still open at the end of the simulation, with the cycle they were last opened.

2. **Detailed Open/Close Sequence for Each Row:**  
   For each row, a sequence of open and close events with their respective cycles.

3. **Chronological Log of Events (Filtered):**  
   All row events, buffer snapshots, and debug events in strict chronological order, with details and tables for buffer contents.

---

## Requirements

- Python 3.x
- `pandas` library

Install pandas if needed:
```sh
pip install pandas
```

---

-----------------------------------------------------------------------------------------------------------------

# build_ramulator2.sh

This script automates the build process for the Ramulator2 simulator.

## What it does

1. Changes directory to `build/`.
2. Runs `cmake ..` to configure the build.
3. Compiles the project using `make -j` (parallel build).
4. Copies the resulting `ramulator2` binary to the project root directory.
5. Returns to the original directory.

## Usage

```sh
bash build_ramulator2.sh
```

Make sure you have a `build/` directory and all necessary dependencies installed before running

-----------------------------------------------------------------------------------------------------------------

# run_all_tests.sh

This script automates running a suite of CloudSuite DRAM benchmarks using Ramulator2, both in single-core and multi-core configurations, and generates summary plots for each run.
It uses example_config_ek.yaml as the run config, but you can change it to your .yaml config file.

## What it does

1. **Sets a unique date-time prefix** for all output files in the session.
2. **Runs single-core benchmarks** (uncomment the lines you want to run).
3. **Runs multi-core benchmarks** (uncomment the lines you want to run).
4. **After each set of runs**, calls `plot_hists.py` to generate PDF plots from the resulting CSVs for both single-core and multi-core runs.

## Usage

```sh
bash run_all_tests.sh
```

- Edit the script to uncomment the specific benchmarks you want to run.
- The script will create result directories and output files with a timestamp prefix for easy organization.

## Output

- Benchmark results and CSVs are stored in `res/` directories with the date prefix.
- Plots are generated as PDFs for each set of runs (single-core and multi-core).

## Requirements

- Ramulator2 and its dependencies
- Python 3 with `matplotlib`, `pandas`, and `numpy`
- The `plot_hists.py` script in `scripts/`

---


3. Results analysis (Unfinished work):
  The outputted metrics produced some anomalies that are not possible for a real memory system:
  Some rows seem to show a re-open interval of e^9 cycles which is longer than the refresh cycle - so clearly this data is not true.
  Another issue that remained unverified is that I haven't managed to load the DRAM device enough to achieve high percentage of DRAM usage, and in most of the 
  runs I got results of <10% device usage and most of the rows were left unopened throught the run.