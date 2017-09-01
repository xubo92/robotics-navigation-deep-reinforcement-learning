

import os, sys
import subprocess
import traci
import random



def generate_routefile(vehicle_domain):

    random.seed(42)  # make tests reproducible
    N = 10000  # number of time steps
    # demand per second from different directions
    pWE = 1. / 5
    pEW = 1. / 6
    pNS = 1. / 15
    lastVeh = 0
    vehNr = 0
    for i in range(N):
        if random.uniform(0, 1) < pWE:

            vehicle_domain.addFull("normal_%i" %vehNr,"left-right",typeID="normal",depart="%i" %i ,departPos="780")
            vehicle_domain.setSpeedMode("normal_%i" %vehNr,0)
            vehNr += 1
            lastVeh = i
        if random.uniform(0, 1) < pEW:

            vehicle_domain.addFull("sporty_%i" % vehNr, "left-right", typeID="sporty", depart="%i" %i,departPos="780")
            vehicle_domain.setSpeedMode("sporty_%i" %vehNr, 0)
            vehNr += 1
            lastVeh = i
        if random.uniform(0, 1) < pNS:

            vehicle_domain.addFull("trailer_%i" % vehNr, "left-right", typeID="trailer", depart= "%i" %i,departPos="780")
            vehicle_domain.setSpeedMode("trailer_%i" %vehNr, 0)
            vehNr += 1
            lastVeh = i




PORT = 8813
sumoBinary = "/usr/bin/sumo-gui"
sumoProcess = subprocess.Popen([sumoBinary, "-c", "crossroad.sumocfg", "--remote-port", str(PORT),"--collision.check-junctions","true","--collision.action","warn"], stdout=sys.stdout, stderr=sys.stderr)



vd = traci._vehicle.VehicleDomain()
traci.init(PORT)
generate_routefile(vd)

step = 0
while step < 10000:
    print "Here we take the control !!"

    if step % 100 == 0:



        vid = "new" + str(step)
        vd.addFull(vid, "cross",typeID="trailer",departPos="780")
        vd.setSpeedMode(vid,0)

    traci.simulationStep()

    step += 1


traci.close()
