#!/usr/bin/env python3

#
# Genera los gráficos de boxplot con la información de los giros
#

import seaborn as sns
import pandas as pd
import matplotlib
matplotlib.rcParams['text.usetex'] = True
from matplotlib import pyplot as plt

data = pd.read_csv('turns-data.csv')
data = data[data.type.isin(['t1', 't2', 'omega1', 'omega2', 'pi3', 'pi4', 'pi5', 'pi6'])]
data['speed'] = data['length']/data['duration']
"""
<class 'pandas.core.frame.DataFrame'>
     iteration  length  duration type
0            1  10.371    26.071   t1
1            2   9.738    23.857   t1
...
"""

plots = [
    #{'variable': 'duration', 'yLabel': 'Time duration', 'unit': ' (s)'}]
    #{'variable': 'length', 'yLabel': 'Length', 'unit': ' (m)'}]
    {'variable': 'speed', 'yLabel': 'Speed', 'unit': ' (m/s)'}]

for plot in plots:

    variable = plot['variable']
    yLabel = plot['yLabel']
    unit = plot['unit']

    sns.set_style("whitegrid")

    """
    my_pal = {"t1": "#2a80b9", "t2": "#50af61", "omega1":"#904fad", "omega2":"#e77e27", 
        "pi3":"#c1392b", "pi4":"#95a5a5", "pi5":"#eace34", "pi6":"#da5290"}
    """
    my_pal = {"t1": "#5DA5DA", "t2": "#60BD68", "omega1":"#FCD838", "omega2":"#FAA43A", 
        "pi3":"#F15854", "pi4":"#B276B2", "pi5":"#96613D", "pi6":"#979797"}

    #ax = sns.boxplot(x='type', y='duration', data=data, palette=my_pal, linewidth=0.5)
    ax = sns.boxplot(x='type', y=variable, data=data, linewidth=1, width=0.6, palette=my_pal)

    plt.title('Comparison of turning types in path ' + variable, fontsize=32)
    plt.xlabel('Turning type', fontsize=24)
    plt.ylabel(yLabel + unit, fontsize=24)

    plt.xticks([*range(0, 8)], [r'$T_{1}$', r'$T_{2}$', r'$\Omega_{1}$', r'$\Omega_{2}$', r'$\Pi_{3}$', r'$\Pi_{4}$', r'$\Pi_{5}$', r'$\Pi_{6}$'], fontsize=12)

    plt.grid(True)
    plt.tight_layout()

    colors = {
        r'$T_{1}$\ (1-jump-t-turn)':my_pal['t1'], 
        r'$T_{2}$\ (2-jumps-t-turn)':my_pal['t2'],
        r'$\Omega_{1}$\ (1-jump-omega-turn)':my_pal['omega1'],
        r'$\Omega_{2}$\ (2-jumps-omega-turn)':my_pal['omega2'],
        r'$\Pi_{3}$\ (3-jumps-pi-turn)':my_pal['pi3'],
        r'$\Pi_{4}$\ (4-jumps-pi-turn)':my_pal['pi4'],
        r'$\Pi_{5}$\ (5-jumps-pi-turn)':my_pal['pi5'],
        r'$\Pi_{6}$\ (6-jumps-pi-turn)':my_pal['pi6']}
    labels = list(colors.keys())
    handles = [plt.Rectangle((0, 0), 1, 1, linewidth=0.8, edgecolor='k', 
        facecolor=colors[label]) for label in labels]
    plt.legend(handles, labels, loc='lower right', fontsize=12)

    figg = matplotlib.pyplot.gcf()
    figg.set_size_inches(12 , 8)

    plt.tight_layout()

    #plt.show()
    plt.savefig('boxplot-full-' + variable + '.pdf')

    plt.clf()
