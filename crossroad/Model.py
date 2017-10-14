import os, sys
import subprocess
import traci
import random
import math
import numpy as np

from keras.models import Sequential
from keras.layers import Dense, Activation,Flatten, Input, merge, Lambda


from keras.initializers import normal, identity
from keras import optimizers

class DQN_for_CROSS:

    def __init__(self,env):

        self.env = env
        self.memory = list()

        self.gamma = 0.9
        self.learning_rate = 0.0001
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.05
        self.create_model()

    def create_model(self):

        model = Sequential()
        model.add(Dense(100,input_dim=198,activation='relu'))
        model.add(Dense(100,activation='relu'))
        model.add(Dense(100,activation='relu'))
        model.add(Dense(12,activation='linear'))
        model.compile(loss='mse',optimizer = optimizers.RMSprop(lr=self.learning_rate))
        self.model = model

    def remember(self,state,action,reward,next_state,done):
        self.memory.append((state,action,reward,next_state,done))


    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return self.env.action_space.sample()
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])  # returns action
