#!/usr/bin/env python3

import os
import re
import subprocess
import sys
from itertools import product

DIR = os.path.dirname(sys.argv[0])
CONFIG = f"{DIR}/../config_cows_ddr5.yaml"

ECHO = ""
# ECHO = "echo"

#
# Early helper
#
def insts_human2hz(hz: str) -> int:
    m = re.match(r"^\s*([0-9]+)\s*([MG]?)(?:Hz)?\s*$", hz, re.IGNORECASE)
    if not m:
        raise ValueError(f"Invalid frequency format: {hz}")

    num = int(m.group(1))
    unit = m.group(2).upper()
    if unit == "G":
        return num * 1_000_000_000
    if unit == "M":
        return num * 1_000_000
    return num


#
# Test configuration: modify these variables to change the test matrix.
#
benchs1 = (
    "data-analytics-core-1 data-caching-core-1 data-serving-core-1 "
    "graph-analytics-core-1 in-memory-analytics-core-1 media-streaming-core-1 "
    "web-search-core-1 web-serving-core-1"
)
benchs8 = (
    "data-analytics-core-8 data-caching-core-8 data-serving-core-8 "
    "graph-analytics-core-8 in-memory-analytics-core-8 media-streaming-core-8 "
    "web-search-core-8 web-serving-core-8"
)
# benchs8 = "data-analytics-core-8"
# benchs = "web-search-core-8"
# benchs = "in-memory-analytics-core-8"

#    __NINSTS__=insts_human2hz("10M") # testing
__NINSTS__=insts_human2hz("100M")
#    __NINSTS__=insts_human2hz("1G")
#    __NINSTS__=insts_human2hz("10G")
#    __NINSTS__=insts_human2hz("20G")

BASE_TEST = {
    "__NINSTS__": [__NINSTS__],
    "__COWS_ENABLE__": ["false"],

    # dummy values
    "__COWS_CACHE2LLC_RATIO__": [1],
    "__COWS_CACHE_ASSOC__": [8],
    "__COWS_ALWAYS_MISS__": ["false"],
    "dir_prefix": "base",
}

COWS_FORCE_MISS_TEST = {
    "__NINSTS__": [__NINSTS__],
    "__COWS_ENABLE__": ["true"],

    # dummy values
    "__COWS_CACHE2LLC_RATIO__": [1],
    "__COWS_CACHE_ASSOC__": [8],
    "__COWS_ALWAYS_MISS__": ["true"],
    "dir_prefix": "base",
}

CURR_TEST = {
    "__NINSTS__": [__NINSTS__],
    "__COWS_ENABLE__": ["true"],
    "__COWS_CACHE2LLC_RATIO__": [1024, 512, 256, 128, 64, 1],
#    "__COWS_CACHE2LLC_RATIO__": [1024, 512, 256, 128, 64, 32, 16, 1],

    "__COWS_CACHE_ASSOC__": [8],
    "__COWS_ALWAYS_MISS__": ["false"],
    "dir_prefix": "stats2",
}

TEST=[
    CURR_TEST,
#    BASE_TEST,
    COWS_FORCE_MISS_TEST
]

ncores = 8


#
# Helper functions to mirror shell script behavior.
#

def get_yaml_value(key: str):
    # Mirror shell behavior: print usage and return failure when key is empty.
    if not key:
        print("usage: get_yaml_value <key>", file=sys.stderr)
        return None

    values = []
    with open(CONFIG, "r", encoding="utf-8") as f:
        for line in f:
            # Equivalent to: grep -v "^ *\#"
            if re.match(r"^ *#", line):
                continue
            # Equivalent to: grep "${key}:"
            if f"{key}:" in line:
                # Equivalent to: awk '{print $2}'
                fields = line.split()
                if len(fields) >= 2:
                    values.append(fields[1])

    # Shell command substitution keeps all command output; join with newlines.
    return "\n".join(values)


def insts_hz2human(hz: str) -> str:
    hz_int = int(hz)

    # The bash version uses an external `calc`; mirror outcomes directly.
    if hz_int >= 1_000_000_000:
        return f"{hz_int / 1e9:.0f}G"
    if hz_int >= 1_000_000:
        return f"{hz_int / 1e6:.0f}M"
    return f"{hz_int}Hz"


def human_to_bytes(size: str) -> int:
    m = re.match(r"^([0-9]+)([KMG]?)B?$", size)
    if not m:
        print(f"Invalid size format: {size}", file=sys.stderr)
        raise ValueError("invalid size")

    num = int(m.group(1))
    unit = m.group(2)
    if unit == "K":
        return num * 1024
    if unit == "M":
        return num * 1024 * 1024
    if unit == "G":
        return num * 1024 * 1024 * 1024
    return num


def flatten_test_matrix(test: dict) -> list[dict]:
    base = {}
    explode_keys = []
    explode_values = []

    for key, value in test.items():
        if isinstance(value, list):
            explode_keys.append(key)
            explode_values.append(value)
        else:
            base[key] = value

    if not explode_keys:
        return [base.copy()]

    cases = []
    for combo in product(*explode_values):
        case = base.copy()
        case.update(dict(zip(explode_keys, combo)))
        cases.append(case)
    return cases


#
# Main function to run the tests based on the configuration and helper functions.
#
def main() -> int:
    if ncores == 1:
        benchs = benchs1
    else:
        benchs = benchs8

    all_tests = []
    for t in TEST:
        if not isinstance(t, dict):
            print("Error: all test matrices must be dictionaries.", file=sys.stderr)
            return 1
        new_tests = flatten_test_matrix(t)
        print(f"new tests: {len(new_tests)}")
        for test in new_tests:
            print(f"\t{test}")
        print()

        all_tests.extend(new_tests)

#    sys.exit(0)

    # ninsts = get_yaml_value("num_expected_insts")
#    cowslat = get_yaml_value("cows_cache_access_latency")
    cowsdramlat = get_yaml_value("cows_dram_latency_on_translation")
    llc_per_core = get_yaml_value("llc_capacity_per_core")
    drampage_kB = get_yaml_value("dram_page_bytes")
    latmul = get_yaml_value("latency_factor")

    llc_bytes = human_to_bytes(llc_per_core) * ncores
    line_size = get_yaml_value("llc_linesize")

    for test_case in all_tests:
        # calc number of entries in cows cache
#        cows_entries = llc_bytes // (int(line_size) * int(test_case["__COWS_CACHE2LLC_RATIO__"]))
#        print(
#            f"llc_bytes={llc_bytes}, line_size={line_size}, ratio={test_case['__COWS_CACHE2LLC_RATIO__']}, "
#            f"cowsassoc={test_case['__COWS_CACHE_ASSOC__']}, cows_entries={cows_entries}"
#        )

        ninsts = insts_hz2human(str(test_case['__NINSTS__']))
        print(f">>>NINSTS={test_case['__NINSTS__']}<<<")


 #       print(
 #           f"cows_enable={test_case['__COWS_ENABLE__']}, cows_always_miss={test_case['__COWS_ALWAYS_MISS__']}, "
 #           f"cows_cache2llc_ratio={test_case['__COWS_CACHE2LLC_RATIO__']}, "
 #       )

        suffix = f"{ninsts}-{ncores}c-llc={llc_per_core}-drampage={drampage_kB}-latmul={latmul}"
        if test_case["__COWS_ENABLE__"] == "false":
            suffix = f"nocows-{suffix}"
        elif test_case["__COWS_ALWAYS_MISS__"] == "true":
            suffix = f"skipcows-{suffix}"
        else:
            cows_suffix = (
                f"cowsratio={test_case['__COWS_CACHE2LLC_RATIO__']}-"
                f"cowsassoc={test_case['__COWS_CACHE_ASSOC__']}-"
                f"cowsdramlat={cowsdramlat}-"
            )
            suffix = f"cows-{suffix}---{cows_suffix}"

        dir_prefix = test_case['dir_prefix']
        dirname = f"{dir_prefix}-{suffix}"

        test_vars = [f"-D{k}={v}" for k, v in test_case.items() if k.startswith("__")]
        for b in benchs.split():
            print(b)
            cmd = [
                f"{DIR}/run.pl",
                "--bench",
                b,
                "--dir",
                dirname,
                "--name",
                b,
                "--config",
                CONFIG,
                *test_vars,
            ]

            print("Running command:", " ".join(cmd))
#            continue

            if ECHO:
                subprocess.Popen([ECHO, *cmd])
            else:
                subprocess.Popen(cmd)

    return 0


if __name__ == "__main__":
    sys.exit(main())
