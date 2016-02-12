#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import logging
import fnmatch
import csv


def GetBeginAndEndTime(data):
    return min(data), max(data)


def GenerateHorizontalBar(data):
    # Determine start and end time
    data_array = np.array(data)
    start_time, end_time = GetBeginAndEndTime(data_array[:, 0])
    # determine segments
    # find activation times
    mask = data_array[:, 5:6]
    print mask.T.shape
    return [], []


def GraphData(data):
    for key in data:
        print "Processing - [%s]" % key
        # Generate bar graph for key value pair
        color, bar_data = GenerateHorizontalBar(data[key])
        #plt.barh(np.arange(len(bar_data)), bar_data, color=color)
    #plt.show()


def main():
    parser = argparse.ArgumentParser(description='Process directory input')
    parser.add_argument('-d', '--dir', type=str, required=True, help='Directory of behavior runtime results')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output that shows graph outputs')
    args = parser.parse_args()

    if not os.path.exists(args.dir):
        logging.error('Directory not found - [%s]' % os.path.abspath(args.dir))
        return -1
    if not os.path.isdir(args.dir):
        logging.error('Error: Expected directory not file')
        return -1

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

    GraphData(data)

if __name__ == "__main__":
    main()
