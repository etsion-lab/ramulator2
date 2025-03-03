#!/usr/bin/env python3

import matplotlib.pyplot as plt
import argparse
import numpy as np
import random

def col_load(file, col_idx):
    ret = np.loadtxt(file, dtype=float, usecols=col_idx, comments='#')
    return ret

def col_sample(nsamples, col):
    if(nsamples == 0):
        return col

    first=col[0]
    last=col[-1]
    col=np.delete(col, [0, len(col)-1])
    nsamples = nsamples - 2

    ret=[]
    ret.append(first)
    for l in np.array_split(col, nsamples):
        ret.append(random.sample(l.tolist(), 1)[0])
    ret.append(last)

    return np.array(ret)

def build_parser():
    parser = argparse.ArgumentParser(
                        prog='plotlines',
                        description='plot lines from columnar files')

    parser.add_argument('-s', '--samples',
                        type=int,
                        default=0,
                        help="how many samples to use from the array")
    parser.add_argument('-f', '--file',
                        required=True,
                        type=argparse.FileType('r'),
                        #nargs='+',
                        action='append',
                        help="file to read from")
    parser.add_argument('-c', '--col',
                        required=True,
                        type=int,
                        help="column to plot (starting at 0)")

    return parser

#
# Main
#
def main():

    parser = build_parser()
    args = parser.parse_args()

    xarray=[ float(v)/(args.samples/100) for v in range(args.samples+1) ]

    for f in args.file:
        col = col_load(f.name, args.col)
        yarray = col_sample(args.samples+1, col)
        #yarray=[ 100.0*v for v in yarray ]

        plt.plot(xarray, yarray)

    plt.xlabel("Run time [%]")
    plt.ylabel("Fraction of entries in LLC needed\nto reqresent cached DRAM pages")
    plt.yticks(np.arange(0.0, 1.01, 0.1))
    plt.grid(visible=True, axis='y', which='both')
    plt.show()

if __name__ == '__main__':
    main()
