

import os, sys
import subprocess
import traci



PORT = 8813
sumoBinary = "/usr/bin/sumo-gui"
sumoProcess = subprocess.Popen([sumoBinary, "-c", "crossroad.sumocfg", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)



vd = traci._vehicle.VehicleDomain()
traci.init(PORT)



step = 0
while step < 10000:
    print "Here we take the control !!"
    if step % 100 == 0:
        vid = "new" + str(step)
        vd.add(vid, "auto route",typeID="trailer")


    traci.simulationStep()

    step += 1


traci.close()
