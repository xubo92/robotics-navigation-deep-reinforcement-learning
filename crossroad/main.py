import os, sys
import subprocess
import traci
import random
import math
import numpy as np
from Env import Intersaction
from Model import DQN_for_CROSS


if __name__ == '__main__':

    Intersac = Intersaction()
    DQN = DQN_for_CROSS(Intersac)

    episode_num = 10000
    ep_idx = 0
    max_timestep = 100

    for ep_idx in range(episode_num):

        cur_state = Intersac.reset()
        print np.shape(cur_state)

        n = 0

        while n <= max_timestep:

            #c_action_idx = DQN.act(cur_state)
            c_action_idx = 2
            next_state,c_reward,done,accident,_ = Intersac.step(c_action_idx)

            print np.shape(next_state)

            DQN.remember(cur_state, c_action_idx, c_reward, next_state, done)


            cur_state = next_state

            if done or accident:
                print "one episode come to end!"
                break

            n += _

    traci.close()