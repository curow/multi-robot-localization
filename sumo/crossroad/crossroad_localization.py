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

seed = 1
randomState = np.random.RandomState(seed)

def printInfo():
    print(F"current time: {traci.simulation.getTime()}")
    print(F"current vehicle list: {traci.vehicle.getIDList()}")
    print(F"departed vehicle list: {traci.simulation.getDepartedIDList()}")
    print(F"arrived vehicle list: {traci.simulation.getArrivedIDList()}")
    ids = traci.vehicle.getIDList()
    for x in ids:
        pos = traci.vehicle.getPosition(x)
        angle = traci.vehicle.getAngle(x)
        print(F"{x}: (x:{pos[0]}, y:{pos[1]}), (angle:{angle / 180 * np.pi})")
    print("\n")

max_range = 10

gps_sigma_x, gps_sigma_y, gps_sigma_angle = 2, 2, np.pi * 10 / 180
odom_sigma_x, odom_sigma_y, odom_sigma_angle = gps_sigma_x / 10, gps_sigma_y / 10, gps_sigma_angle / 10
relative_sigma_x, relative_sigma_y, relative_sigma_angle = gps_sigma_x / 4, gps_sigma_y / 4, gps_sigma_angle / 4

gps_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
                        [gps_sigma_x, gps_sigma_y, gps_sigma_angle]))
odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
                        [odom_sigma_x, odom_sigma_y, odom_sigma_angle]))
relative_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
                        [relative_sigma_x, relative_sigma_y, relative_sigma_angle]))

lag = 10.0
smoother = gtsam_unstable.BatchFixedLagSmoother(lag)

new_factors = gtsam.NonlinearFactorGraph()
new_values = gtsam.Values()
new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
name = {}

def symbol(x, t):
    return (name[x] << 15) + t

def pose(x):
    pos = traci.vehicle.getPosition(x)
    angle = traci.vehicle.getAngle(x) / 180 * np.pi
    return gtsam.Pose2(pos[0], pos[1], angle)

def noise_gps_measurement(x):
    current_pose = pose(x)
    noise_x = randomState.normal(0.0, gps_sigma_x)
    noise_y = randomState.normal(0.0, gps_sigma_y)
    noise_angle = randomState.normal(0.0, gps_sigma_angle)
    gps_measurement = current_pose.compose(gtsam.Pose2(
        noise_x, noise_y, noise_angle
    ))
    return gps_measurement

def noise_odom_measurement(x):
    previous_pose = previous_states[x]
    odom_measurement = previous_pose.between(current_pose)
    noise_x = randomState.normal(0.0, odom_sigma_x)
    noise_y = randomState.normal(0.0, odom_sigma_y)
    noise_angle = randomState.normal(0.0, odom_sigma_angle)
    odom_measurement = odom_measurement.compose(gtsam.Pose2(
        noise_x, noise_y, noise_angle
    ))
    return odom_measurement

def noise_relative_measurement(x, y):
    relative_measurement = pose(x).between(pose(y))
    noise_x = randomState.normal(0.0, relative_sigma_x)
    noise_y = randomState.normal(0.0, relative_sigma_y)
    noise_angle = randomState.normal(0.0, relative_sigma_angle)
    relative_measurement = relative_measurement.compose(gtsam.Pose2(
        noise_x, noise_y, noise_angle
    ))
    return relative_measurement

def euclidean_distance(x, y):
    pos_x = np.array(traci.vehicle.getPosition(x))
    pos_y = np.array(traci.vehicle.getPosition(y))
    return np.linalg.norm(pos_x - pos_y)

previous_states = {}

traci.start(sumoCmd)
time = 0
while time <= 63:
    print(F"time: {time}")
    if traci.vehicle.getIDCount():
        printInfo()
        ids = traci.vehicle.getIDList()
        for x in ids:
            if x not in name:
                name[x] = len(name)

            current_key = symbol(x, time)
            # print(F"current vehicle: {x}")
            # print(F"current name: {name[x]}")
            # print(F"current name after shift: {name[x]<<16}")
            # print(F"current key: {current_key}")
            # print("\n")
            new_timestamps.insert((current_key, time))

            gps_measurement = noise_gps_measurement(x)
            new_factors.push_back(gtsam.PriorFactorPose2(
                current_key, gps_measurement, gps_noise
            ))
            new_values.insert(current_key, gps_measurement)

            if x in previous_states:
                previous_key = symbol(x, time - 1)
                odom_measurement = noise_odom_measurement(x)
                new_factors.push_back(gtsam.BetweenFactorPose2(
                    previous_key, current_key, odom_measurement, odom_noise 
                ))

        for i in ids:
            for j in ids:
                if i == j or euclidean_distance(i, j) > max_range:
                    continue
                relative_measurement = noise_relative_measurement(i, j)
                new_factors.push_back(gtsam.BetweenFactorPose2(
                    symbol(i, time), symbol(j, time), relative_measurement, relative_noise
                ))

        # print(new_factors, new_values, new_timestamps)
        smoother.update(new_factors, new_values, new_timestamps)
        print("after optimization:")
        for x in ids:
            current_key = symbol(x, time)
            print("Timestamp = " + str(time) + ", Vehicle = " + x)
            print(smoother.calculateEstimatePose2(current_key))
        print('\n')

    print(F"end time {time}\n")

    new_timestamps.clear()
    new_values.clear()
    new_factors.resize(0)
            
    traci.simulationStep()
    time += 1 
traci.close()
