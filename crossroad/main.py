import os, sys
import subprocess
import traci
import random
import math
import numpy as np
from Env import Intersaction
from Model import DQN_for_CROSS
import cv2
import matplotlib.pyplot as plt





if __name__ == '__main__':

    Intersac = Intersaction()
    '''
    Intersac.vehicle_domain.addFull(Intersac.c_vid, Intersac.DrivingRoute, typeID="sporty", departPos=Intersac.departPos,
                           arrivalPos=Intersac.arrivalPos)
   
    for i in range(10000):
        traci.simulationStep()
        Intersac.state(Intersac.vehicle_domain)
    '''

    DQN = DQN_for_CROSS(Intersac)
    episode_num = 100000
    ep_idx = 0
    max_timestep = 100

    for ep_idx in range(episode_num):
        loss = 0
        cur_state = Intersac.reset()
        cur_state = np.reshape(cur_state,[1,150*500*1])
        n = 0
        c_action_idx = -1
        while n <= max_timestep:


            if c_action_idx != 4:
                c_action_idx = DQN.act(cur_state)

            next_state, c_reward, done, accident, _ = Intersac.step(c_action_idx)
            print "next state's shape:",next_state.shape


            next_state = np.reshape(next_state,[1,150*500*1])
            DQN.remember(cur_state, c_action_idx, c_reward, next_state, done)

            cur_state = next_state

            if done or accident:
                print "one episode come to end!"
                break

            n += _

            # --------------------------- start replay training -------------------------#

            batch_size = min(DQN.mini_batch_size, len(DQN.memory))
            batches_idx = np.random.choice(len(DQN.memory), batch_size)

            for i in batches_idx:
                replay_c_state, replay_c_action_idx, replay_c_reward, replay_next_state, replay_done = DQN.memory[i]
                print "replay_next_state.shape:",replay_next_state.shape  #[1L,75000L]
                print "replay_c_state.shape:",replay_c_state.shape #[1L,75000L]
                if replay_done:
                    replay_c_dreamValue_c_action = replay_c_reward
                else:
                    # model.predict means forward calculation for getting real label. Since we don't have any label before but to calculate it.
                    # if we have real label(like the situation in Lily's SVM), then we don't need to model.predict here
                    replay_c_dreamValue_c_action = replay_c_reward + DQN.gamma * np.max(DQN.model.predict(replay_next_state)[0])

                temp = DQN.model.predict(replay_c_state)
                temp[0][replay_c_action_idx] = replay_c_dreamValue_c_action
                replay_c_dreamValue_all_actions = temp

                # model.fit means training. replay_c_state : X (input sample); replay_c_expectValue:Y (real label)
                history = DQN.model.fit(replay_c_state, replay_c_dreamValue_all_actions, epochs=1, batch_size=1)


                loss += history.history["loss"][0]
            loss = loss * 1.0 / batch_size
            print("trainning loss at Episode %d, step %d : %.2f" % (ep_idx,n,loss))
            if DQN.epsilon > DQN.epsilon_min:
                DQN.epsilon *= DQN.epsilon_decay

    traci.close()
'''
# test case
if __name__ == '__main__':
    Intersac = Intersaction()
    
    Intersac.vehicle_domain.addFull(Intersac.c_vid, Intersac.DrivingRoute, typeID="sporty", departPos=Intersac.departPos,
                           arrivalPos=Intersac.arrivalPos)

    for i in range(10000):
        traci.simulationStep()
        Intersac.state(Intersac.vehicle_domain)
'''