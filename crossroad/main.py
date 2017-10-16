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

            c_action_idx = DQN.act(cur_state)

            # c_action_idx = 2
            next_state,c_reward,done,accident,_ = Intersac.step(c_action_idx)

            print np.shape(next_state)

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

                if replay_done:
                    replay_c_target = replay_c_reward
                else:
                    replay_c_target = replay_c_reward + DQN.gamma * np.amax(DQN.model.predict(replay_next_state)[0])

                replay_c_expectValue = DQN.model.predict(replay_c_state)
                replay_c_expectValue[0][replay_c_action_idx] = replay_c_target

                DQN.model.fit(replay_c_state, replay_c_expectValue, nb_epoch=1, verbose=0)

            if DQN.epsilon > DQN.epsilon_min:
                DQN.epsilon *= DQN.epsilon_decay

    traci.close()