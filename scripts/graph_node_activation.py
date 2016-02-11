#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import logging
import fnmatch
import csv


def main():
    parser = argparse.ArgumentParser(description='Process directory input')
    parser.add_argument('-d', '--dir', type=str, required=True, help='Directory of behavior runtime results')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output that shows graph outputs')
    args = parser.parse_args()

    if not os.path.exists(args.dir):
        logging.error('Directory not found - [%s]' % os.path.abspath(args.dir))
    if not os.path.isdir(args.dir):
        logging.error('Error: Expected directory not file')

    print 'Loading files...'
    files = [f for f in os.listdir(args.dir) if os.path.isfile(os.path.join(args.dir, f))]
    # Generate data dictionary to store information
    data = dict()
    for file in files:
        if fnmatch.fnmatch(file, '*.csv'):
            print 'Loading [%s]' % file
            with open(os.path.join(args.dir, file), 'rb') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=',')
                data[file] = list(spamreader)

    print 'Processing Data...'

if __name__ == "__main__":
    main()
