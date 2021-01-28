#!/usr/bin/env python3

#
# Muestra resultados estadísticos de los giros
#

import pandas as pd

data = pd.read_csv('turns-data.csv')
data['speed'] = data['length']/data['duration']
del data['iteration']

l = ['t1', 't2', 'omega1', 'omega2', 'pi3', 'pi4', 'pi5', 'pi6']
medians = {}
for t in l:
    description = data[data['type'] == t].describe()
    median = description['duration']['50%']
    medians[t] = "%.3f" % median
    print("Turn", t)
    print(description)
    print()
    
print('Time durations medians:')
for median in medians:
    print(median, medians[median])
