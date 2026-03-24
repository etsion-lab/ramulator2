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
    return xs, ys


def main():
    parser = argparse.ArgumentParser(
        prog='plot-lines',
        description='Plot lines from columnar data files')
    parser.add_argument('-k', '--key-cols',
                        required=True,
                        type=parse_k,
                        metavar='x,y',
                        help='1-based column indices for X and Y (e.g. -k 1,2)')
    parser.add_argument('-l', '--legend-regex',
                        default=None,
                        metavar='PATTERN',
                        help='regex with () group to extract legend label from file path')
    parser.add_argument('-t', '--title',
                        default=None,
                        metavar='TITLE',
                        help='plot title')
    parser.add_argument('-xmin', type=float, default=None, metavar='XMIN')
    parser.add_argument('-xmax', type=float, default=None, metavar='XMAX')
    parser.add_argument('-ymin', type=float, default=None, metavar='YMIN')
    parser.add_argument('-ymax', type=float, default=None, metavar='YMAX')
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

    if args.xmin is not None or args.xmax is not None:
        plt.xlim(left=args.xmin, right=args.xmax)
    if args.ymin is not None or args.ymax is not None:
        plt.ylim(bottom=args.ymin, top=args.ymax)

    plt.grid(visible=True, axis='y')
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
