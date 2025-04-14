#!/usr/bin/env python3

import sys

def read_file(fname):
    pages=dict()
    numpages=0
    with open(fname, encoding="utf-8") as f:
        for line in f:
            s=line.split()
            type=s[-2]
            hash=s[-1]

            if type.startswith("zero"):
                continue

            if not hash in pages:
                pages[hash]=0

            pages[hash] = pages[hash] + 1
            numpages = numpages + 1

    return (pages, numpages)

def categorize_strings(list_a, list_b):
    """
    Categorizes strings from two lists into three groups:
    - Only in list a
    - Only in list b
    - In both list a and list b

    Args:
        list_a: The first list of strings.
        list_b: The second list of strings.

    Returns:
        A tuple containing three lists:
        - strings_only_in_a: Strings present only in list_a.
        - strings_only_in_b: Strings present only in list_b.
        - strings_in_both: Strings present in both list_a and list_b.
    """

    set_a = set(list_a)
    set_b = set(list_b)

    only_in_a = list(set_a - set_b)
    only_in_b = list(set_b - set_a)
    in_both = list(set_a & set_b)

    return only_in_a, only_in_b, in_both


def find_dedups(bname, stats):
    dedups=dict()
    bpages = stats[bname]["pages"];
    bnumpages = stats[bname]["numpages"];

    print(f"find dedup on {bname=}")
    for b in sorted(stats.keys()):
#        print(f"other bench={b}")
        other_pages=stats[b]["pages"]
        other_numpages=stats[b]["numpages"]

        (in_bench, in_other, common) = categorize_strings(bpages.keys(), other_pages.keys())
        b_common=0
        for k in common:
            b_common = b_common + bpages[k]

        other_common=0
        for k in common:
            other_common = other_common + other_pages[k]

        ncommon = b_common + other_common
        ntotal = bnumpages + other_numpages
        frac = ncommon / ntotal
        dedups[b] = { "b_common": b_common, "other_common": other_common, "common": ncommon, "total": ntotal, "frac": frac }

    return dedups
#
# Main
#
def main():
    if len(sys.argv) == 1:
        print(f"Usage: {sys.argv[0]}: <mem-file1> [mem-file2] [mem-file3]...")
        sys.exit(1)

    files=sys.argv[1:]

    stats = dict()
    for f in files:
        dirs = f.split("/")
        bname = dirs[-2]

        print(f"Reading bench {bname}")
        (pages, numpages) = read_file(f)

        keys=len(pages.keys())
        print(f"{bname=}, {numpages=}, {keys=}, frac={keys/numpages}")
        
        stats[bname] = { "pages": pages, "numpages": numpages }

    for b in sorted(stats.keys()):
        dedups = find_dedups(b, stats)

        str=""
        for bd in sorted(dedups.keys()):
            frac = dedups[bd]["frac"]
            b_common = dedups[bd]["b_common"]
            other_common = dedups[bd]["other_common"]
            str = str + f"{frac:.2}\t\t"

        print(f"{b:<30}\t{str}")

        
    
if __name__ == '__main__':
    main()
