# -*- coding: utf-8 -*-

import os, sys
import subprocess
import traci
import random
import math
import numpy as np
from PIL import Image,ImageDraw

class action_space:
    def __init__(self,action_num):
        if not action_num:
            print "dimisition size should not be 0"
        else:

            self.action_idx = np.arange(action_num)

    def sample(self):
        length = len(self.action_idx)
        if length:

            return np.random.randint(0,length)
        else:
            print "dimisition size should not be 0"


class Intersaction:

    global config
    config = dict()
    config['PORT'] = 8813
    #config['SumoBinary'] = "/usr/bin/sumo-gui"  # for linux
    #config['SumoBinary'] = "/usr/local/bin/sumo-gui" # for Mac
    config['SumoBinary'] = "C:\\Program Files (x86)\\Sumo\\bin\\sumo-gui"
    def __init__(self):

        subprocess.Popen([config['SumoBinary'], "-c", "crossroad.sumocfg", "--remote-port", str(config['PORT']), "--collision.check-junctions", "true","--collision.action", "teleport","--step-length","0.2"], stdout=sys.stdout, stderr=sys.stderr)

        self.vehicle_domain = traci._vehicle.VehicleDomain()
        self.simu_domain = traci._simulation.SimulationDomain()
        self.vehicletype_domain = traci._vehicletype.VehicleTypeDomain()
        self.departPos = "750"
        self.arrivalPos = "20"
        self.DrivingRoute = "cross"

        self.cur_state = None
        self.cur_reward = None


        self.Audi_viewrange_axis0 = 100
        self.Audi_viewrange_axis1 = 30
        self.Audi_viewmap = Image.new('L',(self.Audi_viewrange_axis0,self.Audi_viewrange_axis1),255)

        self.done = False
        self.accident = False
        self.action_space = action_space(12)

        self.goal_speed = 0

        traci.init(config['PORT'])
        self.c_vid = "Audi L3"
        self.generate_flow(self.vehicle_domain)




    def reset(self):

        if self.accident:
            self.vehicle_domain.remove(self.c_vid)

        #print self.vehicle_domain.getIDList()

        if self.c_vid not in self.vehicle_domain.getIDList():

            self.vehicle_domain.addFull(self.c_vid, self.DrivingRoute, typeID="sp", departPos=self.departPos,arrivalPos=self.arrivalPos)

        self.vehicle_domain.setSpeedMode(self.c_vid, 0)
        self.vehicle_domain.setSpeed(self.c_vid,0)

        self.goal_speed = 0

        traci.simulationStep()

        cord_set = self.gen_MapPositionsWithIdx(self.c_vid,self.vehicle_domain)
        self.cur_state = self.gen_state(cord_set,self.vehicle_domain.getIDList(),self.vehicle_domain)

        self.done = False
        self.accident = False

        return self.cur_state


    # state: 30 * 100 * 1, grey scale image
    # range: -50 ~ +50; 0 ~ 30; Do not consider the direction change of vehicle's head
    def state(self, vehicle_domain):
        vehicle_list = self.vehicle_domain.getIDList()
        print vehicle_list
        Audi_x, Audi_y = vehicle_domain.getPosition(self.c_vid)
        Audi_interval_axis0 = (Audi_x - self.Audi_viewrange_axis0,Audi_x + self.Audi_viewrange_axis0)
        Audi_interval_axis1 = (Audi_y,Audi_y + self.Audi_viewrange_axis1)
        valid_vehicles = dict()
        for vid in vehicle_list:
            x, y = vehicle_domain.getPosition(vid)
            print "vid:%s, vid.x:%d, vid.y:%d" % (vid, x, y)
            vehicle_L = self.vehicletype_domain.getLength(vehicle_domain.getTypeID(vid))/2
            vehicle_W = self.vehicletype_domain.getWidth(vehicle_domain.getTypeID(vid))/2

            Ax = (x - vehicle_L - Audi_interval_axis0[0])
            Bx = (x + vehicle_L - Audi_interval_axis0[0])
            Cx = (x - vehicle_L - Audi_interval_axis0[1])
            Dx = (x + vehicle_L - Audi_interval_axis0[1])
            Ay = (y - vehicle_W - Audi_interval_axis1[0])
            By = (y + vehicle_W - Audi_interval_axis1[0])
            Cy = (y - vehicle_W - Audi_interval_axis1[1])
            Dy = (y + vehicle_W - Audi_interval_axis1[1])

            if vid != self.c_vid:
                if Ax * Bx < 0 :
                    if Ay * By < 0:
                        valid_vehicles[vid] = [Audi_interval_axis0[0], x + vehicle_L,Audi_interval_axis1[0],y + vehicle_W]
                    elif Cy * Dy < 0:
                        valid_vehicles[vid] = [Audi_interval_axis0[0], x + vehicle_L,y - vehicle_W,Audi_interval_axis1[1]]
                    elif Ay * Dy < 0:
                        valid_vehicles[vid] = [Audi_interval_axis0[0], x + vehicle_L,y - vehicle_W,y + vehicle_W]

                elif Cx * Dx < 0:
                    if Ay * By < 0:
                        valid_vehicles[vid] = [x - vehicle_L,Audi_interval_axis0[1],Audi_interval_axis1[0],y + vehicle_W]
                    elif Cy * Dy < 0:
                        valid_vehicles[vid] = [x - vehicle_L,Audi_interval_axis0[1],y - vehicle_W,Audi_interval_axis1[1]]
                    elif Ay * Dy < 0:
                        valid_vehicles[vid] = [x - vehicle_L,Audi_interval_axis0[1],y - vehicle_W,y + vehicle_W]

                elif Ax * Dx < 0:
                    if Ay * By < 0:
                        valid_vehicles[vid] = [x - vehicle_L, x + vehicle_L,Audi_interval_axis1[0],y + vehicle_W]
                    elif Cy * Dy < 0:
                        valid_vehicles[vid] = [x - vehicle_L, x + vehicle_L,y - vehicle_W,Audi_interval_axis1[1]]
                    elif Ay * Dy < 0:
                        valid_vehicles[vid] = [x - vehicle_L, x + vehicle_L,y - vehicle_W,y + vehicle_W]

        print "valid_vehicles:",valid_vehicles
        for key in valid_vehicles:
            self.draw_rect(valid_vehicles[key],self.Audi_viewmap)

        self.Audi_viewmap.show()
    # Generate other traffic flows
    def generate_flow(self, vehicle_domain):

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
                vehicle_domain.addFull("normal_%i" % vehNr, "left-right", typeID="normal", depart="%i" % i,
                                       departPos="750")
                vehicle_domain.setSpeedMode("normal_%i" % vehNr, 0)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW:
                vehicle_domain.addFull("sporty_%i" % vehNr, "left-right", typeID="sporty", depart="%i" % i,
                                       departPos="750")
                vehicle_domain.setSpeedMode("sporty_%i" % vehNr, 0)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNS:
                vehicle_domain.addFull("trailer_%i" % vehNr, "left-right", typeID="trailer", depart="%i" % i,
                                       departPos="750")
                vehicle_domain.setSpeedMode("trailer_%i" % vehNr, 0)
                vehNr += 1
                lastVeh = i

    def step(self,action_idx):

        steps = (action_idx % 3 + 1)  # steps
        timesteps = steps * 0.2  # seconds

        mod = action_idx / 3

        if mod == 0:
            # accel is 2m/s^2
            self.goal_speed = self.goal_speed + 2 * timesteps
            # timesteps * 1000 --> ms
            self.vehicle_domain.slowDown(self.c_vid,self.goal_speed,timesteps*1000)
        elif mod == 1:
            self.goal_speed = self.goal_speed - 2 * timesteps
            if self.goal_speed < 0:
                self.goal_speed = 0
            self.vehicle_domain.slowDown(self.c_vid,self.goal_speed,timesteps*1000)
        elif mod == 2:
            self.vehicle_domain.slowDown(self.c_vid,self.goal_speed,timesteps*1000)

        #print "goal speed:",self.goal_speed
        #print "current time:",self.simu_domain.getCurrentTime()
        #print "time step period:",timesteps*1000


        traci.simulationStep(self.simu_domain.getCurrentTime()+timesteps*1000)



        print "current vehicles:", self.vehicle_domain.getIDList()

        teleport_list = self.simu_domain.getEndingTeleportIDList()
        print("teleport list:", teleport_list)
        arrived_list = self.simu_domain.getArrivedIDList()
        print("arrived list:", arrived_list)

        if len(teleport_list):
            print("collisions happened!")
            self.accident = True
            self.cur_reward = -10

        elif self.c_vid in arrived_list:
            print("successful arrival!")
            self.done = True
            self.cur_reward = 1
        else:
            self.cur_reward = -0.01

        if not self.done:
            # 实际速度上比goal speed少了0.4m/s是对的，getSpeed函数只能获得上一时间点的速度
            print "audi L3 speed at last step:", self.vehicle_domain.getSpeed(self.c_vid)
            cord_set = self.gen_MapPositionsWithIdx(self.c_vid, self.vehicle_domain)
            self.cur_state = self.gen_state(cord_set, self.vehicle_domain.getIDList(), self.vehicle_domain)
            print cord_set
        else:
            self.cur_state = np.ones((6,11,3))

        return self.cur_state,self.cur_reward,self.done,self.accident,steps

    
    def draw_rect(self,rect,Image):
        draw = ImageDraw.Draw(Image)
        # rect -> [rectX_low, rectX_top, rectY_low, rectY_top]
        for x in range(rect[0], rect[1]):
            for y in range(rect[2],rect[3]):
                draw.rectangle(((rect[0],rect[2]),(rect[1],rect[3])),fill='black')

















#*************** back up for test **************#

'''
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
'''
'''
PORT = 8813
#sumoBinary = "/usr/local/bin/sumo-gui"  # on Mac
sumoBinary = "/usr/bin/sumo-gui"  # on linux
sumoProcess = subprocess.Popen([sumoBinary, "-c", "crossroad.sumocfg", "--remote-port", str(PORT),"--collision.check-junctions","true","--collision.action","teleport"],stdout=sys.stdout, stderr=sys.stderr)



vd = traci._vehicle.VehicleDomain()
sd = traci._simulation.SimulationDomain()

traci.init(PORT)

generate_flow(vd)

vid = "Audi L3"
vd.addFull(vid, "cross", typeID="trailer", departPos="780", arrivalPos="780")
vd.setSpeedMode(vid, 0)


step = 0

while step < 10000:
    print "Here we take the control !!"

/*****************************************************************/
    if "reach goal || 100 steps gone || collisions happen":

        "first remove the auto car "
        "then add a new auto car"
/*****************************************************************/  

    teleport_list = sd.getEndingTeleportIDList()
    print("teleport list:",teleport_list)
    arrived_list = sd.getArrivedIDList()
    print("arrived list:",arrived_list)

    if vid in teleport_list:
        collision_t = sd.getCurrentTime()
        print("collisions happened!")
        print("collision time: %d" % collision_t)
        vd.remove(vid)
        vd.addFull(vid, "cross", typeID="trailer", departPos="780", arrivalPos="780")
        vd.setSpeedMode(vid, 0)

    elif vid in arrived_list:

        vd.addFull(vid, "cross", typeID="trailer", departPos="780", arrivalPos="780")
        vd.setSpeedMode(vid, 0)
        
    elif not step % 100:
        vd.remove(vid)
        vd.addFull(vid, "cross", typeID="trailer", departPos="780", arrivalPos="780")
        vd.setSpeedMode(vid, 0)

    #print vd.getPosition(vid)
    #print len(vd.getIDList())
    traci.simulationStep()
    

    step += 1

traci.close()

'''