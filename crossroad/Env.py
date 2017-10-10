

import os, sys
import subprocess
import traci
import random
import math
import numpy as np
import fcntl


class Intersaction:

    global config
    config = dict()
    config['PORT'] = 8813
    config['SumoBinary'] = "/usr/bin/sumo-gui"

    def __init__(self):

        subprocess.Popen([config['SumoBinary'], "-c", "crossroad.sumocfg", "--remote-port", str(config['PORT']), "--collision.check-junctions", "true","--collision.action", "warn"], stdout=sys.stdout, stderr=sys.stderr)

        self.vehicle_domain = traci._vehicle.VehicleDomain()
        self.departPos = "780"
        self.arrivalPos = "780"
        self.DrivingRoute = "cross"

        self.cur_state = None
        self.cur_reward = None
        self.action_space = np.arange(0,12,1)

        self.generate_flow(self.vehicle_domain)
        self.c_vid = 'new' + '0'
        self.vehicle_domain.addFull(self.c_vid, self.DrivingRoute, typeID="trailer", departPos=self.departPos, arrivalPos=self.arrivalPos)
        self.vehicle_domain.setSpeedMode(self.c_vid, 0)

        self.cur_speed = self.vehicle_domain.getSpeed(self.c_vid)

        traci.init(config['PORT'])

    def reset(self):

        self.vehicle_domain.remove(self.c_vid)
        self.c_vid = 'new' + '0'
        self.vehicle_domain.addFull(self.c_vid, self.DrivingRoute, typeID="trailer", departPos=self.departPos,arrivalPos=self.arrivalPos)
        self.vehicle_domain.setSpeedMode(self.c_vid, 0)

        cord_set = self.gen_MapPositionsWithIdx(self.c_vid,self.vehicle_domain)
        self.cur_state = self.gen_state(cord_set,self.vehicle_domain.getIDList(),self.vehicle_domain)
        return self.cur_state

    def step(self,action_idx):

        timesteps = action_idx / 3 + 1
        mod = action_idx / 3
        if mod == 0:
            self.vehicle_domain.slowDown(self.c_vid,self.cur_speed + 2 * timesteps,timesteps)
        elif mod == 1:
            self.vehicle_domain.slowDown(self.c_vid,self.cur_speed - 2 * timesteps,timesteps)
        elif mod == 2:
            self.vehicle_domain.slowDown(self.c_vid,self.cur_speed,timesteps)


        traci.simulationStep(timesteps)


        cord_set = self.gen_MapPositionsWithIdx(self.c_vid, self.vehicle_domain)
        self.cur_state = self.gen_state(cord_set, self.vehicle_domain.getIDList(), self.vehicle_domain)

        return self.cur_state,self.cur_reward


    def generate_flow(self,vehicle_domain):
    
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
    
    
    def gen_MapPositionsWithIdx(self,vid,vehicle_domain):
    
        x,y = vehicle_domain.getPosition(vid)
        x = round(x)
        y = round(y)
    
        coordinate_set = set()
    
        for i in range(x-90,x+90+18,18):
            for j in range(y-0,y+20+4,4):
                coordinate_set.add((i,j,(y-j)/4+5,(i-x)/18+5))
    
        return coordinate_set
    
    
    
    def gen_state(self,cord_set,vehicle_list,vd):
    
        state = np.zeros((6,11,3))
        for x,y,row_idx,col_idx in cord_set:
            for vid in vehicle_list:
                if self.isOccupied(x,y,vid,vd):
                    angle = vd.getAngle(vid)
                    velocity = vd.getSpeed(vid)
                    collision_t = ""  # remain to be calculated later...
                    state[row_idx,col_idx] = [angle,velocity,collision_t]
    
    
        return state
    
    
    
    def isOccupied(self,x,y,vid,vd):
    
        v_bumper_pos = vd.getPosition(vid)
        v_bumper_pos_x = v_bumper_pos[0]
        v_bumper_pos_y = v_bumper_pos[1]
    
        v_width = vd.getWidth(vid)
        v_length = vd.getLength(vid)
        v_angle = vd.getAngle(vid)
        sin_angle = math.sin(v_angle)
        cos_angle = math.cos(v_angle)
    
        v_rect_ax = v_bumper_pos_x - 1/2 * v_width * sin_angle
        v_rect_ay = v_bumper_pos_y + 1/2 * v_width * cos_angle
    
        v_rect_bx = v_bumper_pos_x + 1/2 * v_width * sin_angle
        v_rect_by = v_bumper_pos_y - 1/2 * v_width * cos_angle
    
        v_rect_dx = v_bumper_pos_x - v_length * cos_angle + 1/2 * v_width * sin_angle
        v_rect_dy = v_bumper_pos_y - v_length * sin_angle - 1/2 * v_width * cos_angle
    
        bax = v_rect_bx - v_rect_ax
        bay = v_rect_by - v_rect_ay
        dax = v_rect_dx - v_rect_ax
        day = v_rect_dy - v_rect_ay
    
        if ((x - v_rect_ax) * bax + (y - v_rect_ay) * bay < 0.0): return False
        if ((x - v_rect_bx) * bax + (y - v_rect_by) * bay > 0.0): return False
        if ((x - v_rect_ax) * dax + (y - v_rect_ay) * day < 0.0): return False
        if ((x - v_rect_dx) * dax + (y - v_rect_dy) * day > 0.0): return False
    
        return True



def generate_flow(vehicle_domain):
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
            vehicle_domain.addFull("normal_%i" % vehNr, "left-right", typeID="normal", depart="%i" % i, departPos="780")
            vehicle_domain.setSpeedMode("normal_%i" % vehNr, 0)
            vehNr += 1
            lastVeh = i
        if random.uniform(0, 1) < pEW:
            vehicle_domain.addFull("sporty_%i" % vehNr, "left-right", typeID="sporty", depart="%i" % i, departPos="780")
            vehicle_domain.setSpeedMode("sporty_%i" % vehNr, 0)
            vehNr += 1
            lastVeh = i
        if random.uniform(0, 1) < pNS:
            vehicle_domain.addFull("trailer_%i" % vehNr, "left-right", typeID="trailer", depart="%i" % i,
                                   departPos="780")
            vehicle_domain.setSpeedMode("trailer_%i" % vehNr, 0)
            vehNr += 1
            lastVeh = i



def nonblocking(fd):
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)


PORT = 8813
#sumoBinary = "/usr/local/bin/sumo-gui"  # on Mac
sumoBinary = "/usr/bin/sumo-gui"  # on linux
sumoProcess = subprocess.Popen([sumoBinary, "-c", "crossroad.sumocfg", "--remote-port", str(PORT),"--collision.check-junctions","true","--collision.action","warn"],stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,close_fds=True)



vd = traci._vehicle.VehicleDomain()


traci.init(PORT)

generate_flow(vd)

step = 0


'''
output,err_msg=sumoProcess.communicate()
print output
print err_msg
'''
while step < 10000:
    print "Here we take the control !!"

    '''
    if "reach goal || 100 steps gone || collisions happen":

        "first remove the auto car "
        "then add a new auto car"
    '''

    vid = "new" + str(step)
    vd.addFull(vid, "cross",typeID="trailer",departPos="780",arrivalPos="780")
    vd.setSpeedMode(vid,0)



    print vd.getPosition("new0")
    print len(vd.getIDList())
    traci.simulationStep()

    step += 1





traci.close()