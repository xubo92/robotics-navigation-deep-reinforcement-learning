# Robotics-Naigation-Deep-Reinforcement-Learning

## Instructions

# why do I create this repository?
I am focusing on the robotics navigation problem with reinforcement learning methods. All the time people consider SLAM methods as the kernel solution for navigation purpose of robot. However, SLAM method has a few difficulty parts that are not well solved for a long time. I'm considering and trying to find a distinctive way, with the help of deep reinforcement learning method, to see if there are any improvement in robotics navigation.

# what is in this repository?
Currently, I am writting the code based on the paper ["Navigation intersactions with Autonomous vehicles using Deep Reinforcement Learning"](http://xueshu.baidu.com/s?wd=paperuri%3A%288096d7729767e358d7308ee6a1bb902d%29&filter=sc_long_sign&tn=SE_xueshusource_2kduw22v&sc_vurl=http%3A%2F%2Farxiv.org%2Fpdf%2F1705.01196&ie=utf-8&sc_us=17616306211134359942). This paper provided an efficient strategy to navigate safety through unsignaled intersactions.

- **basic** : Include the elementary parts of a 'SUMO' map 

  **IMPORTANT**: SUMO is the short of "Simulation of Urban Mobility". It is a software which help you creat any kinds of traffic simulation scenes quickly. 
  
  **IMPORTANT**: You'd better read the tutorials of SUMO first in order to understand the content below. Tutorials are here: [SUMO TUTORIALS](http://sumo.dlr.de/wiki/Tutorials).Apart from that, the blog is great for your quick scan on SUMO's utility.
  
  There are five parts below:
  * [car.rou.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/basic/car.rou.xml) : Defined by user. You have to define the car flow on certain route or place in order to simulate the traffic scene.
  * [edges.edg.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/basic/edges.edg.xml) : Defined by user. To build a road net(see [road.net.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/basic/road.net.xml)) , you have to define the every edge as you wish.
  * [nodes.nod.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/basic/nodes.nod.xml) : Defined by user. To build a road net, you also have to define every node in order to connect edges together to be a map.
  * [road.net.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/basic/road.net.xml) : Not defined by user, but generated by the file [edges.edg.xml] and [nodes.nod.xml]. As for the command. see and find it in [SUMO TUTORIALS](http://sumo.dlr.de/wiki/Tutorials)
  * [cfg.sumocfg.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/basic/cfg.sumocfg.xml) : Defined by user. This file contains simulation parameters mainly for the process of entire simulation.
  
- **crossroad** : Include the source code for reappearance of result in paper above.
  **IMPORTANT** : 'Traci' is the short for "Traffic Control Interface". It is a library used for the communication with SUMO based on TCP protocol in order to modify and recieve values in simulation on-line. We apply Traci here since we need to interfere with simulation procedure('Environment' in reinforcement learing) to choose actions or stop the episode('action' and 'episode' are terms in reinforcement learing). To get familiar with Traci first, you'd better check [Traci Tutorials](http://www.sumo.dlr.de/userdoc/TraCI/Protocol.html)
  There are four parts now:
  ![](file:///C:\Users\lvxub\Desktop\RL论文\programImage.png)
   + Traffic map
     - [crossroad.net.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/crossroad/crossroad.net.xml) : a complete road net
     - [crossroad.rou.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/crossroad/crossroad.rou.xml) : a convenient way to define the motion trace of cars. See [SUMO TUTORIALS](http://sumo.dlr.de/wiki/Tutorials)
     - [crossroad.sumocfg.xml](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/crossroad/crossroad.sumocfg) : As usual, the parameters file for simulation circle.
   + Deep reinforcement learning model
     - [Model.py](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/crossroad/Model.py) : Implemented a basic DQN model based on the paper ["Human level control through deep reinforcement learning"](https://www.nature.com/articles/nature14236.pdf). This model serves as a complicated function to get policy output via image features input.
   + Environment setting
     - [Env.py](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/crossroad/Env.py) : Implemented environment logic such as how to convert information from SUMO to the necessary states in Markov Decision Process.
   + reinforcement learning algorithm and main function for training
     - [Main.py](https://github.com/lvlvlvlvlv/Robotics-Navigation-Deep-Reinforcement-Learning/blob/master/crossroad/main.py) : Implemented Q-learning algorithm embedded in whole training procedure. You only need to execute this file to start the training.
  
  


