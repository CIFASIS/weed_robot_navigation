#!/usr/bin/env python3

#
# Generates graphs of APE and RPE errors comparing the robot's trajectory with the ideal one.
#

import subprocess
from subprocess import PIPE
import seaborn as sns
import pandas as pd
import matplotlib
matplotlib.rcParams['text.usetex'] = True
from matplotlib import pyplot as plt
import numpy as np
import matplotlib.lines as mlines

def plotError(pathType, errorType, poseRelation):

    blue = '#5DA5DA'
    green = '#60BD68'
    red = '#F15854'
    lightGrey = '#d8d8d8'
    grey = '#4c4c4c'

    pathTypeName = pathType['name']
    errorTypeName = errorType['name']
    poseRelationName = poseRelation['name']
    resultName = pathTypeName + '-' + errorTypeName + '-' + poseRelationName

    subprocess.run('evo_' + errorTypeName + ' tum tum/' + pathTypeName + '-reference-path.tum tum/' + pathTypeName + '-cut-path.tum' 
        + ' --pose_relation ' + poseRelationName + ' --save_results ' + resultName + '.npz', 
        shell=True, stdout=PIPE, stderr=PIPE)

    results = np.load(resultName + '.npz')
    timestamps = results['timestamps.npz']
    timestamps -= min(timestamps)
    errors = results['error_array.npz']
    """
    all the timestamps and errors of the route
    timestamps: array([ 17.239,  17.34 ,  17.44 , ..., 599.585, 599.686, 599.787])
    errors: array([0.00333301, 0.00398368, 0.00458133, ..., 0.022053  , 0.02140115, 0.02076639])
    len(timestamps) == len(errors)
    """
    #print(len(timestamps), len(errors))

    sns.set_style("whitegrid")

    fig, ax1 = plt.subplots()

    ax1.plot(timestamps, errors, color=grey, linewidth=2.0)

    minTime = min(timestamps)
    maxTime = max(timestamps)
    # mean
    mean = np.mean(errors)
    ax1.plot([minTime, maxTime], [mean, mean], color=green, linewidth=2.0)
    # median
    median = np.median(errors)
    ax1.plot([minTime, maxTime], [median, median], color=red, linewidth=2.0)
    # rmse
    rmse = np.sqrt(np.mean(errors**2))
    ax1.plot([minTime, maxTime], [rmse, rmse], color=blue, linewidth=2.0)
    # std
    std = np.std(errors)
    bottom = mean - std / 2
    top = mean + std / 2
    ax1.fill_between([minTime, maxTime], [bottom, bottom], [top, top], color=lightGrey, linewidth=2.0)

    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)
    plt.xlim(minTime, maxTime)

    plt.title(poseRelation['pretty'] + ' ' + errorType['pretty'] + ' for ' + pathType['pretty'], fontsize=22, pad=20.0)
    plt.xlabel('Timestamp (s)', fontsize=14)
    plt.ylabel(errorType['pretty'] + ' (' + poseRelation['unit'] + ')', fontsize=16)

    plt.grid(True)
    plt.tight_layout()

    lines = {errorType['pretty']:grey, 'RMSE':blue, 'Mean':green, 'Median':red}
    boxes = {'Std Dev':lightGrey}
    labels = list(lines.keys()) + list(boxes.keys())
    linesHandles = [mlines.Line2D([], [], linewidth=2, color=lines[label]) for label in lines.keys()]
    boxesHandles = [plt.Rectangle((0, 0), 1, 1, linewidth=0, facecolor=boxes[label]) for label in boxes.keys()]
    handles = linesHandles + boxesHandles
    plt.legend(handles, labels, loc='upper right', fontsize=12)

    figg = matplotlib.pyplot.gcf()
    figg.set_size_inches(8, 6)

    plt.tight_layout()

    #plt.show()
    plt.savefig(resultName + '.pdf')
    plt.savefig(resultName + '.png')
    plt.close()

pathTypes = [
    {'name': 'optimal', 'pretty': 'optimal path'},
    {'name': 'three-jumps', 'pretty': '3 jumps pattern'},
    {'name': 'trivial', 'pretty': 'trivial path with reverse'},
    {'name': 'trivial-omega', 'pretty': r'trivial path with $\Omega$-turns'}]

errorTypes = [
    {'name': 'ape', 'pretty': 'ATE'},
    {'name': 'rpe', 'pretty': 'RPE'}]

poseRelations = [
    {'name': 'trans_part', 'pretty': 'Translational', 'unit': 'm'},
    {'name': 'angle_rad', 'pretty': 'Rotational', 'unit': 'rad'}]

for pathType in pathTypes:
    for errorType in errorTypes:
        for poseRelation in poseRelations:
            plotError(pathType, errorType, poseRelation)
