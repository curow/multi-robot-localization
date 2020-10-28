#!/usr/bin/env python
# coding: utf-8

import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
import numpy as np
import gtsam
import gtsam_unstable

sumoBinary = "/usr/local/bin/sumo"
sumoCmd = [sumoBinary, "-c", "cross.sumocfg"]

def printInfo():
    print(F"current time: {traci.simulation.getTime()}")
    print(F"current vehicle list: {traci.vehicle.getIDList()}")
    print(F"departed vehicle list: {traci.simulation.getDepartedIDList()}")
    print(F"arrived vehicle list: {traci.simulation.getArrivedIDList()}")
    ids = traci.vehicle.getIDList()
    for x in ids:
        pos = traci.vehicle.getPosition(x)
        angle = traci.vehicle.getAngle(x)
        print(F"{x}: (x:{pos[0]}, y:{pos[1]}), (angle:{angle})")
    print("\n")

lag = 10.0
smoother = gtsam_unstable.BatchFixedLagSmoother(lag)

new_factors = gtsam.NonlinearFactorGraph()
new_values = gtsam.Values()
new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()

sigma_x, sigma_y, sigma_angle = 2, 2, np.pi * 10 / 180
max_range, sigma_range = 10, 0.5
gps_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
                        [sigma_x, sigma_y, sigma_angle]))
odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
                        [sigma_x / 10, sigma_y / 10, sigma_angle / 10]))

name = {}
def symbol(x, t):
    return gtsam.symbol(name[x], t)

def pose(x):
    pos = traci.vehicle.getPosition(x)
    angle = traci.vehicle.getAngle(x) / 180 * np.pi
    return gtsam.Pose2(pos[0], pos[1], angle)

previous_states = {}

traci.start(sumoCmd)
time = 0
while time <= 63:
    if traci.vehicle.getIDCount():
        printInfo()
        ids = traci.vehicle.getIDList()
        for x in ids:
            if x not in name:
                name[x] = len(name)
            current_key = symbol(x, time)
            new_timestamps.insert((current_key, time))
            current_pose = pose(x)
            new_values.insert(current_key, current_pose)
            if x in previous_states:
                previous_key = symbol(x, time - 1)
                previous_pose = previous_states[x]
                odom_measurement = previous_pose.between(current_pose)
                odom_measurement = odom_measurement.compose()
                new_factors.push_back(gtsam.BetweenFactorPose2(
                    previous_key, current_key, , odom_noise 
                ))
            
    traci.simulationStep()
    time += 1 
traci.close()
