#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.colors as colors
import matplotlib.cm as cmx
import argparse
import os
import fnmatch
import csv
import pickle
import pdb
from math import sqrt
import matplotlib

SPINE_COLOR = 'gray'


def latexify(fig_width=None, fig_height=None, columns=2):
    """Set up matplotlib's RC params for LaTeX plotting.
    Call this before plotting a figure.

    Parameters
    ----------
    fig_width : float, optional, inches
    fig_height : float,  optional, inches
    columns : {1, 2}
    """

    # code adapted from http://www.scipy.org/Cookbook/Matplotlib/LaTeX_Examples

    # Width and max height in inches for IEEE journals taken from
    # computer.org/cms/Computer.org/Journal%20templates/transactions_art_guide.pdf

    assert(columns in [1,2])

    if fig_width is None:
        fig_width = 3.39 if columns==1 else 6.9 # width in inches

    if fig_height is None:
        golden_mean = (sqrt(5)-1.0)/2.0    # Aesthetic ratio
        fig_height = fig_width*golden_mean # height in inches

    MAX_HEIGHT_INCHES = 8.0
    if fig_height > MAX_HEIGHT_INCHES:
        print("WARNING: fig_height too large:" + fig_height + 
              "so will reduce to" + MAX_HEIGHT_INCHES + "inches.")
        fig_height = MAX_HEIGHT_INCHES

    params = {'backend': 'ps',
              'text.latex.preamble': ['\usepackage{gensymb}'],
              'axes.labelsize': 14, # fontsize for x and y labels (was 10)
              'axes.titlesize': 22,
              'font.size': 14, # was 10
              'legend.fontsize': 14, # was 10
              'xtick.labelsize': 14,
              'ytick.labelsize': 14,
              'text.usetex': True,
              'figure.figsize': [fig_width,fig_height],
              'font.family': 'serif'
    }

    matplotlib.rcParams.update(params)


def format_axes(ax):

    for spine in ['top', 'right']:
        ax.spines[spine].set_visible(False)

    for spine in ['left', 'bottom']:
        ax.spines[spine].set_color(SPINE_COLOR)
        ax.spines[spine].set_linewidth(0.5)

    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    for axis in [ax.xaxis, ax.yaxis]:
        axis.set_tick_params(direction='out', color=SPINE_COLOR)

    return ax

state_color_map = {'000' : [0.8, 0.9, 0.9, 1.0],
                   '001' : [0.75, 0.6, 0.6, 1.0],
                   '101' : [0.2, 0.3, 0.25, 1.0],
                   '011' : [0.3, 0.4, 0.6, 1.0]}
def GraphData(data, order):
  latexify(16, 8)
  viridis = plt.get_cmap('viridis')
  cnorm = colors.Normalize(vmin=0, vmax=100)
  scalarmap = cmx.ScalarMappable(norm=cnorm, cmap=viridis)

  # Generate subplots for activation potential
  order_list = ['' for x in xrange(len(order))]
  for key in order:
    # for each data item it is necessary to create a subplot. 
    # This needs to follow the order provided
    try:
      order_list[order[key]] = [item for item in data if key in item][0]
    except IndexError:
      order_list[order[key]] = None

  # remove some of the behaviors
  order_list = [key for key in order_list if not (('THEN' in key) or ('AND' in key) or ('OR' in key))]
  order_list = [key for key in order_list if '005' in key or '006' in key or '008' in key]
  # get max value!!
  max_value = 0.0
  min_time = float('inf')
  for key in order_list:
    array_data = np.array([[float(j) for j in i] for i in data[key]])
    max_value = max(np.max(array_data[:,4]), max_value)
    min_time = min(np.min(array_data[:,0]), min_time)

  labels = {
    'THEN_0_1_001_state_Data_.csv'  : 'THEN\_0',
    'PLACE_3_1_002_state_Data_.csv' : 'PLACEMAT',
    'AND_3_1_003_state_Data_.csv'   : 'AND\_0',
    'OR_3_1_004_state_Data_.csv'    : 'OR\_0',
    'PLACE_3_1_005_state_Data_.csv' : 'SPOON',
    'PLACE_3_1_006_state_Data_.csv' : 'FORK',
    'THEN_0_1_007_state_Data_.csv'  : 'THEN\_1',
    'PLACE_3_1_008_state_Data_.csv' : 'KNIFE',
    'PLACE_3_1_009_state_Data_.csv' : 'WINEGLASS',
    'PLACE_3_1_010_state_Data_.csv' : 'CUP',
    'PLACE_3_1_011_state_Data_.csv' : 'SODA',
    'PLACE_3_1_012_state_Data_.csv' : 'PLATE',
    'PLACE_3_1_013_state_Data_.csv' : 'BOWL',
  }
  # user ordered list to  build each plot
  fig = plt.figure()
  for index, key in enumerate(order_list):
    array_data = np.array([[float(j) for j in i] for i in data[key]])
    sub = plt.subplot(len(order_list), 1, index+1)
    if index == 0:
      ax = plt.gca()
      ax.set_title('Activation Potential')
      # plt.title('Activation Potential')
    plt.plot(array_data[:,0] - min_time, array_data[:,4])
    plt.ylim([-0.01, max_value+0.1])
    plt.xlim(left=0)
    plt.ylabel(labels[key])
    plt.xlabel('t(s)')
    if index != len(order_list) - 1:
      plt.setp(sub.get_xticklabels(), visible=False)
  ax = plt.gca()
  format_axes(ax)
  plt.show()

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
  # Load ordering dictionary

  with open('table_setting_demo_node_graph_order.txt', 'r') as file:
    order = pickle.loads(file.read())

  # Generate data dictionary to store information
  data = dict()

  for file in files:
    if file != 'remote_mutex.csv' and fnmatch.fnmatch(file, '*.csv'):
      print 'Loading [{}]'.format(file)
      with open(os.path.join(args.dir, file), 'rb') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',')
        data[file] = list(spamreader)
  print 'Processing Data...'

  GraphData(data, order)


if __name__ == '__main__':
  main()