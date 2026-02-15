#!/usr/bin/env python3
"""Bootstrap script for comparing statistics.

Uses two positional args: arg1 is the stat name, arg2 is the test name.
Extend `main` with the actual comparison logic.
"""

import sys
from pathlib import Path

import yaml
import os
import argparse

BASEDIR="/scratch/yetsion/cows/tools/ramulator2/res"
BASETST="base-8c-1G-nocows-no-zerocopy-accel"

def load_ramulator_yaml(path: str) -> dict:
    """Load YAML starting at 'Frontend:' and stop before trailing timing logs."""
    #print(f"Loading RAMulator YAML from {path}")

    file_path = Path(path)
    data = file_path.read_text().splitlines()
    start_idx = None
    end_idx = len(data)
    for idx, line in enumerate(data):
        if start_idx is None and line.lstrip().startswith("Frontend:"):
            start_idx = idx
            data[idx] = "  " + "Frontend:"
        elif "Command being timed" in line:
            end_idx = idx
            break
        else:
            data[idx] = "  " + line

    if start_idx is None:
        return None

    yaml_content = "\n".join(data[start_idx:end_idx])

#    print(f"Loading YAML from {path}, lines {start_idx} to {end_idx}")
#    print(yaml_content)
    ret = yaml.safe_load(yaml_content)

    ret['Frontend']['llc_total_hits'] = ret['Frontend'].get('llc_total_access', 1) - ret['Frontend'].get('llc_total_misses', 1)
#    print("llc_total_hits: ", ret['Frontend']['llc_total_hits'])
    ret['Frontend']['cows_misses_on_llc_hit_ratio'] = ret['Frontend'].get('cows_misses_on_llc_hit', 0) / ret['Frontend'].get('llc_total_hits', 1)
#    print("cows_misses_on_llc_hit_ratio: ", ret['Frontend']['cows_misses_on_llc_hit_ratio'])
    ret['Frontend']['cows_misses_on_llc_miss_ratio'] = ret['Frontend'].get('cows_misses_on_llc_miss', 0) / ret['Frontend'].get('llc_total_misses', 1)
#    print("cows_misses_on_llc_miss_ratio: ", ret['Frontend']['cows_misses_on_llc_miss_ratio'])


    return ret

def compute_ipc(data: dict) -> float:
    """Compute IPC and add it to the Frontend section."""
    frontend = data.get('Frontend', {})
    total_cycles = sum(
        v for k, v in frontend.items()
        if k.startswith('cycles_recorded_post_warmup_core_')
    )
    total_insts = sum(
        v for k, v in frontend.items()
        if k.startswith('insts_retired_post_warmup_core_')
    )

    ipc = total_insts / total_cycles
    frontend['ipc_total'] = ipc
    return ipc

def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Compare stats between baseline and test runs"
    )
    parser.add_argument("stat_name", help="Stat name to compare")
    parser.add_argument("test_name", help="Test directory name")
    parser.add_argument("-b", "--bench", help="Specific benchmark to compare")
    parser.add_argument("-s", "--size", help="Benchmark size filter")
    parser.add_argument("-c", "--cores", help="Core count filter")
    parser.add_argument("-t", "--base", help="Core count filter")

    args = parser.parse_args(argv)

    statname = args.stat_name
    testname = args.test_name

    if args.base is not None:
        global BASETST
        BASETST = args.base

    if not statname.strip():
        print("Error: stat name cannot be empty.", file=sys.stderr)
        return 1
    if not testname.strip():
        print("Error: test name cannot be empty.", file=sys.stderr)
        return 1


    benchs = [f for f in os.listdir(BASEDIR + "/" + testname) if f.endswith('-8') or f.endswith('-1')]

    base_path = Path(BASEDIR) / BASETST
    bench_path = Path(BASEDIR) / testname

    print(f" statname: {statname}\n testname: {testname}\n base_path: {base_path}\n bench_path: {bench_path}")

    for b in benchs:
        base_file = base_path / b / "out"
        bench_file = bench_path / b / "out"

        print(f"processing {b}...\r", end="", flush=True)

        base_dict = load_ramulator_yaml(str(base_file))
        bench_dict = load_ramulator_yaml(str(bench_file))
        if base_dict is None or bench_dict is None:
            print(f"Missing stats for benchmark {b}.")
            continue

        if 'ipc_total' not in base_dict.get('Frontend', {}):
            compute_ipc(base_dict)
        if 'ipc_total' not in bench_dict.get('Frontend', {}):
            compute_ipc(bench_dict)

        base_stat=base_dict['Frontend'].get(statname, 1)
        bench_stat=bench_dict['Frontend'].get(statname)
#        print(f"bench {b}: Base {statname}: {base_stat}, Bench {statname}: {bench_stat}")
        ratio = bench_stat / base_stat if base_stat != 0 else float('inf')
        print(f"{b:<30} bench/base: {ratio:12.3f} (base: {base_stat}, bench: {bench_stat})")



if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
    # Progress print; put inside the for b in benchs loop
    print(f"processing {b}...", end="", flush=True)
