#!/usr/bin/env python3

#
# Generates TUM files for the ideal trajectory to be used as reference
#

q = 0.7071067811865476

for pathType in ['optimal', 'three-jumps', 'trivial', 'trivial-omega']:

    tum = open('tum/' + pathType + '-full-path.tum', 'r')
    tumCut = open('tum/' + pathType + '-cut-path.tum', 'w')
    tumRef = open('tum/' + pathType + '-reference-path.tum', 'w')
    nroTracks = 15
    start = (nroTracks - 1) * 2.08 / -2.0
    tracks = [start + 2.08 * x for x in range(0, 15)]
    for line in tum:
        if 'timestamp' in line:
            tumCut.write(line)
            tumRef.write(line)
            continue
        lineSplit = line.split()
        timestamp = float(lineSplit[0])
        ty = float(lineSplit[2])
        if ty < -10.0 or ty > 10.0:
            continue
        tumCut.write(line)
        tx = float(lineSplit[1])
        error = float('inf')
        newTx = 0
        for track in tracks:
            if abs(tx - track) < error:
                error = abs(tx - track)
                newTx = track
        #print(tx, newTx)
        tz = float(lineSplit[3])
        qx = float(lineSplit[4])
        qy = float(lineSplit[5])
        qz = float(lineSplit[6])
        if qz > 0:
            qz = q
        else:
            qz = -q
        qw = q
        refLine = ' '.join(["{:.18e}".format(timestamp), 
            "{:.18e}".format(newTx), "{:.18e}".format(ty), "{:.18e}".format(tz),
            "{:.18e}".format(qx), "{:.18e}".format(qy), "{:.18e}".format(qz), "{:.18e}".format(qw)]) + '\n'
        tumRef.write(refLine)

    tum.close()
    tumCut.close()
    tumRef.close()
