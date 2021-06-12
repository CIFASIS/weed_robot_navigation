#!/usr/bin/env python3

#
# Ejecuta la simulación y recolecta los datos para las pruebas de giros
#

import subprocess
from subprocess import PIPE
from time import time
from time import sleep
import signal
import os
import sys
import datetime

def formatTime(sec):
    return str(datetime.timedelta(seconds=round(sec)))

def runNavigation(name, track, omega):
    start = time()
    print("        Navegación... ", end = '', flush=True)

    fStdout = open('log/' + name + '-stdout.log', 'w')
    fStderr = open('log/' + name + '-stderr.log', 'w')
    turningTestScript = os.path.expanduser('~/catkin_ws/src/weed_robot/field_test.sh')
    if omega:
        omegaStr = ' -o'
    else:
        omegaStr = ''
    pNav = subprocess.Popen(turningTestScript + ' -n -t "[3, ' + str(track + 2) + ']" -p "-x -14.56 -y 1.0 -Y 1.570796327" -l "-10.0"' + omegaStr,
        stdout=fStdout, stderr=fStderr, shell=True, preexec_fn=os.setsid, universal_newlines=True)

    pBag = subprocess.Popen('rosbag record -O bag/' + name + '-full.bag ' + 
        '/amcl_pose /move_base/GlobalPlanner/plan /move_base/TebLocalPlannerROS/local_plan /waypoint/waypoints /ackermann_steering_controller/cmd_vel ' +
        '__name:=my_bag', 
        stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)

    sleep(5)
    process = subprocess.Popen('rostopic echo /goal/state', shell=True, stdout=PIPE, stderr=PIPE, universal_newlines=True)
    j = 0
    while j < 120:
        sleep(1)
        j += 1
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
    tum = open('tum/' + name + '-full-path.tum', 'r')
    tumShort = open('tum/' + name + '-short-path.tum', 'w')
    for line in tum:
        lineSplit = line.split()
        timestamp = float(lineSplit[0])
        x = float(lineSplit[1])
        y = float(lineSplit[2])
        if y < 10:
            continue
        tumShort.write(line)
    tum.close()
    tumShort.close()
    subprocess.run(
        'evo_traj tum tum/' + name + '-full-path.tum --plot_mode xy --save_plot plot/' + name + '-full-path.pdf', 
        shell=True, stdout=PIPE, stderr=PIPE)
    subprocess.run(
        'evo_traj tum tum/' + name + '-short-path.tum --plot_mode xy --save_plot plot/' + name + '-short-path.pdf', 
        shell=True, stdout=PIPE, stderr=PIPE)

    end = time()
    print("fin " + formatTime(end - start))

def collectData(name):
    start = time()
    print("        Recolección de datos... ", end = '', flush=True)

    p = subprocess.Popen(
        'evo_traj tum tum/' + name + '-short-path.tum', universal_newlines=True, shell=True, stdout=PIPE, stderr=PIPE)
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
dataFile = open('turns-data.csv', 'a')
#dataFile.write('iteration,length,duration,type\n')
n = 30
l = [('omega', 1, True), ('omega', 2, True), ('pi', 3, None), ('pi', 4, None), ('pi', 5, None), ('pi', 6, None)]
print("Se correrán " + str(n) + " iteraciones de " + str(len(l)) + " tipos de giro distintos")
for t in l:
    turnType = t[0] + str(t[1])
    print("Giro tipo " + turnType)
    for i in range(30, n + 1):
        print("    Iteración " + str(i))
        name = turnType  + '-' + str(i)
        runNavigation(name, t[1] + 1, t[2])
        sleep(1)
        registerPath(name)
        data = collectData(name)
        print('            Path length: ' + data[0] + 'm')
        print('            Path duration: ' + data[1] + 's')
        dataFile.write(str(i) + ',' + data[0] + ',' + data[1] + ',' + turnType + '\n')
        dataFile.flush()
        print()
subprocess.run('find plot/ -type f -name "*_rpy_view.pdf" -exec rm -f {} \\;', shell=True, stdout=PIPE, stderr=PIPE)
subprocess.run('find plot/ -type f -name "*_xyz_view.pdf" -exec rm -f {} \\;', shell=True, stdout=PIPE, stderr=PIPE)
dataFile.close()
globalEnd = time()
print("Fin " + formatTime(globalEnd - globalStart))
