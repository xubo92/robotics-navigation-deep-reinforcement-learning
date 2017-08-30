

import os, sys
import subprocess
import traci
import traci.constants as tc
import traci._vehicle as tv
PORT = 8813
sumoBinary = "/usr/bin/sumo-gui"
sumoProcess = subprocess.Popen([sumoBinary, "-c", "crossroad.sumocfg", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)



traci.init(PORT)
step = 0
AutoCar = tv.VehicleDomain()
AutoCar.setStop("Auto","-gneE0_0")
while step < 10000:
    print "here we take the control"
    traci.simulationStep()

    step += 1


traci.close()
