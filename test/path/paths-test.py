#!/usr/bin/env python3

#
# Ejecuta la simulación y recolecta los datos para las pruebas de trayectorias
#

import subprocess
from subprocess import PIPE
from time import time
from time import sleep
import signal
import os
import sys
import datetime
import fileinput

def formatTime(sec):
    return str(datetime.timedelta(seconds=round(sec)))

def runNavigation(name, sequence, omega):
    start = time()
    print("        Navegación... ", flush=True)

    fStdout = open('log/' + name + '-stdout.log', 'w')
    fStderr = open('log/' + name + '-stderr.log', 'w')
    fieldTestScript = os.path.expanduser('~/catkin_ws/src/weed_robot/field_test.sh')
    if omega:
        omegaStr = ' -o'
    else:
        omegaStr = ''
    pNav = subprocess.Popen(fieldTestScript + ' -n -t "' + str(sequence) + '"' + omegaStr,
        stdout=fStdout, stderr=fStderr, shell=True, preexec_fn=os.setsid, universal_newlines=True)

    pBag = subprocess.Popen('rosbag record -O bag/' + name + '-full.bag /amcl_pose __name:=my_bag', 
        stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)

    sleep(5)
    process = subprocess.Popen('rostopic echo /goal/state', shell=True, stdout=PIPE, stderr=PIPE, universal_newlines=True)
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            if "data:" in output:
                print(output.split('"')[1])
            if "Finished!" in output:
                break
    process.terminate()

    os.killpg(os.getpgid(pNav.pid), signal.SIGTERM)
    fStdout.close()
    fStderr.close()
    
    subprocess.run('rosnode kill /my_bag', shell=True, stdout=PIPE, stderr=PIPE)

    end = time()
    print("fin " + formatTime(end - start))

def registerPath(name):
    start = time()
    print("        Cálculo de trayectoria... ", end = '', flush=True)

    subprocess.run('evo_traj bag bag/' + name + '-full.bag /amcl_pose --save_as_tum', shell=True, stdout=PIPE, stderr=PIPE)
    subprocess.run('mv amcl_pose.tum tum/' + name + '-full-path.tum', shell=True, stdout=PIPE, stderr=PIPE)
    subprocess.run(
        'evo_traj tum tum/' + name + '-full-path.tum --plot_mode xy --save_plot plot/' + name + '-full-path.pdf', 
        shell=True, stdout=PIPE, stderr=PIPE)

    end = time()
    print("fin " + formatTime(end - start))

def collectData(name):
    start = time()
    print("        Recolección de datos... ", end = '', flush=True)

    p = subprocess.Popen(
        'evo_traj tum tum/' + name + '-full-path.tum', universal_newlines=True, shell=True, stdout=PIPE, stderr=PIPE)
    while True:
        line = p.stdout.readline()
        if 'infos:' in line:
            break
    length = line.split(', ')[1].split('m')[0]
    duration = line.split(', ')[2].split('s')[0]

    end = time()

    print("fin " + formatTime(end - start))
    return length, duration


globalStart = time()
dataFile = open('data.csv', 'w')
dataFile.write('iteration,length,duration,type\n')

l = [
    #{'name':'other', 'sequence':[1, 4, 7, 10, 13, 14, 11, 8, 5, 2, 3, 6, 9, 12, 15]}
    {'name':'other', 'sequence':[1, 4, 7, 10, 13, 15, 12, 9, 6, 3, 2, 5, 8, 11, 14]},
    {'name':'trivial', 'sequence':[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]},
    {'name':'optimal', 'sequence':[1, 4, 7, 3, 6, 2, 5, 8, 11, 14, 10, 13, 9, 12, 15]}
]
#l = [{'name':'optimal', 'sequence':[1, 2]}]
i = 1
for t in l:
    for j in range(1, 5):
        name = t['name'] + str(j)
        print("Secuencia " + name)
        sequence = [x + 2 for x in t['sequence']]
        runNavigation(name, sequence, False)
        sleep(1)
        registerPath(name)
        data = collectData(name)
        #print('            Path length: ' + data[0] + 'm')
        #print('            Path duration: ' + data[1] + 's')
        dataFile.write(str(i) + ',' + data[0] + ',' + data[1] + ',' + name + '\n')
        i += 1
        print()
#subprocess.run('find plot/ -type f -name "*_rpy_view.pdf" -exec rm -f {} \\;', shell=True, stdout=PIPE, stderr=PIPE)
#subprocess.run('find plot/ -type f -name "*_xyz_view.pdf" -exec rm -f {} \\;', shell=True, stdout=PIPE, stderr=PIPE)
dataFile.close()
globalEnd = time()
print("Fin " + formatTime(globalEnd - globalStart))
