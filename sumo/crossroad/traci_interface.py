import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "/usr/local/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "cross.sumocfg"]

def printInfo():
    print(F"current vehicle list: {traci.vehicle.getIDList()}")
    print(F"departed vehicle list: {traci.simulation.getDepartedIDList()}")
    print(F"arrived vehicle list: {traci.simulation.getArrivedIDList()}")
    print("\n")

import traci
traci.start(sumoCmd)
while not traci.vehicle.getIDCount():
    traci.simulationStep()
printInfo()
while traci.vehicle.getIDCount():
    traci.simulationStep()
    printInfo()
traci.close()
