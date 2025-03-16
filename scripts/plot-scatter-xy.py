#!/usr/bin/env python3

import matplotlib.pyplot as plt
import argparse
import numpy as np
import random

def col_load(file, col_idx):
    ret = np.loadtxt(file, dtype=float, usecols=col_idx, comments='#')
    return ret

def sampleXY(nsamples, xarray, yarray):
    if(nsamples == 0):
        return col

    firstX=xarray[0]
    firstY=xarray[0]
    lastX=xarray[-1]
    lastY=xarray[-1]

    xarray=np.delete(xarray, [0, len(xarray)-1])
    yarray=np.delete(yarray, [0, len(yarray)-1])
    nsamples = nsamples - 2

    ret_xarray=[]
    ret_yarray=[]
    ret_xarray.append(firstX)
    ret_yarray.append(firstY)
    base_idx = 0
    for l in np.array_split(xarray, nsamples):
        idx = random.sample(range(len(l)), 1)[0]
        ret_xarray.append(xarray[base_idx])
        ret_yarray.append(yarray[base_idx])

        base_idx = base_idx + len(l)

    ret_xarray.append(lastX)
    ret_yarray.append(lastY)

    return np.array(ret_xarray, ret_yarray)

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
    parser.add_argument('-x', '--colX',
                        required=True,
                        type=int,
                        help="column for X axis (starting at 0)")
    parser.add_argument('-y', '--colY',
                        required=True,
                        type=int,
                        help="column for Y axis (starting at 0)")
    parser.add_argument('-o', '--output',
                        help="output file to save to")

    return parser

#
# Main
#
def main():

    parser = build_parser()
    args = parser.parse_args()

    for f in args.file:
        xarray = col_load(f.name, args.colX)
        yarray = col_load(f.name, args.colY)

        if(args.samples != 0):
            (xarray, yarray) = sampleXY(args.samples, xarray, yarray)

        plt.scatter(xarray, yarray)

#    plt.xlabel("Run time [%]")
#    plt.ylabel("Fraction of entries in LLC needed\nto reqresent cached DRAM pages")
#    plt.yticks(np.arange(0.0, 1.01, 0.1))
#    plt.grid(visible=True, axis='y', which='both')
    if args.output is not None:
        print(f"Saving image to {args.output}")
        plt.savefig(args.output)
    else:
        plt.show()

if __name__ == '__main__':
    main()
