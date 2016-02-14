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

def GetState(array):
    result = ""
    for elem in array:
        result += str(int(elem))
    return result


def StateChange(a_arr, b_arr):
    if GetState(a_arr) != GetState(b_arr):
        return True
    return False


def GetStateColor(state):
    color = [1.0, 1.0, 1.0, 1.0]
    if state == '000':
        color = [0.5, 0.5, 1.0, 1.0]
    elif state == '001':
        color = [0.9, 0.1, 0.9, 1.0]
    elif state == '101':
        color = [0.3, 0.4, 0.8, 1.0]
    return color

def GenerateHorizontalBar(data):
    # Determine start and end time
    data_array = np.array([[float(j) for j in i] for i in data])
    start_time, end_time = GetBeginAndEndTime(data_array[:, 0])
    # determine segments
    # find activation times
    # build bitmask
    state_array = data_array[:,0:4]
    state_array[:,3] = state_array[:,3] > 0.18

    plot_state_segments = list()
    plot_state_segments.append((state_array[0][0], GetState(state_array[0][1:4])))
    for i in xrange(len(state_array)):
        # time, active, done, activation_level
        if i > 0:
            # Check for state chagnes
            if StateChange(state_array[i-1][1:4], state_array[i][1:4]):
                plot_state_segments.append((state_array[i][0], GetState(state_array[i][1:4])))
    plot_state_segments.append((state_array[-1][0], '000'))

    color = []
    segments = []
    for i in xrange(len(plot_state_segments)-1):
        segments.append(plot_state_segments[i+1][0] - plot_state_segments[i][0])
        color.append(GetStateColor(plot_state_segments[i][1]))

    # generate color and index
    return segments, color


def GraphData(data):
    bar_width = 0.4
    segment_list = list()
    max_length = -1
    for key in data:
        print "Processing - [%s]" % key
        # Generate bar graph for key value pair
        bar_info = GenerateHorizontalBar(data[key])
        if len(bar_info[0]) > max_length:
            max_length = len(bar_info[0])
        segment_list.append(bar_info)
    # graph segments
    for i in xrange(max_length):
        # extract row
        for elem in segment_list:
            if i < len(elem)
        data = [j[0][i] for j in segment_list]
        color = [j[1][i] for j in segment_list]
        plt.barh(np.arange(0, len(data)*bar_width, bar_width), data, bar_width, color=color)
    # plt.barh(np.arange(0, len(bar_data)*bar_width, bar_width), bar_data, bar_width, color=color)
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
        if file != 'remote_mutex.csv':
            if fnmatch.fnmatch(file, '*.csv'):
                print 'Loading [%s]' % file
                with open(os.path.join(args.dir, file), 'rb') as csvfile:
                    spamreader = csv.reader(csvfile, delimiter=',')
                    data[file] = list(spamreader)

    print 'Processing Data...'

    GraphData(data)

if __name__ == "__main__":
    main()
