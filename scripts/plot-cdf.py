#!/usr/bin/env python3

import argparse
import re
import os
import numpy as np
import matplotlib.pyplot as plt


def parse_k(value):
    parts = value.split(',')
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("-k must be in x,y format (e.g. -k 1,2)")
    try:
        x, y = int(parts[0]), int(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("-k values must be integers") from exc
    if x < 1 or y < 1:
        raise argparse.ArgumentTypeError("-k column indices are 1-based and must be >= 1")
    return (x, y)


def extract_label(filepath, pattern):
    if pattern is None:
        return os.path.basename(filepath)
    m = re.search(pattern, filepath)
    if m and m.lastindex and m.lastindex >= 1:
        return m.group(1)
    return os.path.basename(filepath)


def load_xy(filepath, xcol, ycol, label):
    print(f"Reading X axis for '{label}'...")
    xs = np.loadtxt(filepath, comments='#', usecols=(xcol - 1,))
    print(f"Reading Y axis for '{label}'...")
    ys = np.loadtxt(filepath, comments='#', usecols=(ycol - 1,))
    if xs.ndim == 0:
        xs, ys = xs.reshape(1), ys.reshape(1)
    if xs[0] != 0.0 or ys[0] != 0.0:
        xs = np.insert(xs, 0, 0.0)
        ys = np.insert(ys, 0, 0.0)
    return xs, ys


def main():
    parser = argparse.ArgumentParser(
        prog='plot-cdf',
        description='Plot CDFs from columnar data files')
    parser.add_argument('-k', '--key-cols',
                        required=True,
                        type=parse_k,
                        metavar='x,y',
                        help='1-based column indices for X and CDF-Y (e.g. -k 1,2)')
    parser.add_argument('-l', '--legend-regex',
                        default=None,
                        metavar='PATTERN',
                        help='regex with () group to extract legend label from filename')
    parser.add_argument('-t', '--title',
                        default=None,
                        metavar='TITLE',
                        help='plot title')
    parser.add_argument('-xmax',
                        type=float,
                        default=None,
                        metavar='XMAX',
                        help='truncate x axis at this value')
    parser.add_argument('files',
                        nargs='+',
                        metavar='FILE',
                        help='input data files')

    args = parser.parse_args()
    xcol, ycol = args.key_cols

    for filepath in args.files:
        label = extract_label(filepath, args.legend_regex)
        xs, ys = load_xy(filepath, xcol, ycol, label)
        plt.plot(xs, ys, label=label)

    if args.title is not None:
        plt.title(args.title)
    if args.xmax is not None:
        plt.xlim(right=args.xmax)
    plt.ylim(0, 1.05)
    plt.yticks(np.arange(0, 1.01, 0.1))
    plt.grid(visible=True, axis='y', which='both')
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
