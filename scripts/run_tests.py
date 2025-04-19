import os
import subprocess
import sys
from pathlib import Path

# Constants
RAMULATOR_SCRIPT = "/scratch/elior.k/ramulator2/scripts/run_ek.pl"
PLOT_SCRIPT = "/scratch/elior.k/ramulator2/scripts/plot_hists.py"
RES_DIR = "/scratch/elior.k/ramulator2/res"
CONFIG_TEMPLATE = "/scratch/elior.k/ramulator2/example_config_ek.yaml"

def run_test(test_name, bench_list, output_dir):
    """
    Run a single test with the given test name and bench list.
    """
    # Prepare the command with multiple --bench flags
    base_dir = os.path.dirname(output_dir)  # Use the base directory for --dir
    cmd = [
        "perl", RAMULATOR_SCRIPT,
        "--name", test_name,
        "--config", CONFIG_TEMPLATE,
        "--dir", base_dir  # Pass only the base directory
    ]
    for bench in bench_list:
        cmd.extend(["--bench", bench])  # Add a separate --bench flag for each benchmark

    print(f"Running test: {test_name}")
    print(f"Command: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

def collect_csv_files(output_dir):
    """
    Collect all CSV files from the output directory.
    """
    csv_files = []
    for root, _, files in os.walk(output_dir):
        for file in files:
            if file.endswith(".csv"):
                csv_files.append(os.path.join(root, file))
    return csv_files

def plot_histograms(csv_files):
    """
    Call the plot_hists.py script on the given CSV files.
    """
    cmd = ["python3", PLOT_SCRIPT] + csv_files
    print(f"Plotting histograms for CSV files: {csv_files}")
    subprocess.run(cmd, check=True)

def main():
    if len(sys.argv) < 3:
        print(f"Usage: python {sys.argv[0]} <output_dir> <trace_list1> [<trace_list2> ...]")
        sys.exit(1)

    # Parse arguments
    output_dir = os.path.join(sys.argv[1])
    print(output_dir)
    trace_lists = sys.argv[2:]

    # Create the main output directory
    os.makedirs(output_dir, exist_ok=True)

    # Run tests for each trace list
    for i, trace_list in enumerate(trace_lists):
        test_name = f"test_{i+1}"
        bench_list = trace_list.split(",")  # Assume trace lists are comma-separated
        run_test(test_name, bench_list, output_dir)

    # Collect all CSV files
    csv_files = collect_csv_files(output_dir)

    # Plot histograms
    if csv_files:
        plot_histograms(csv_files)
    else:
        print("No CSV files found. Skipping histogram plotting.")

if __name__ == "__main__":
    main()