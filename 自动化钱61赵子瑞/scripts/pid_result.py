import os
import sys
lib_path = os.path.abspath(os.path.join(sys.path[0], '..'))
sys.path.append(lib_path)
import gym
import matplotlib.pyplot as plt
from src.PID import *
import math
import time

Ctl = PID()
Ctl.setKp(9)
Ctl.setKi(4)
Ctl.setKd(0.85)
Ctl.setSampleTime(0.05)
graph = []

env = gym.make('Pendulum-v0')
for i_episode in range(10):
    observation = env.reset()
    Ctl.clear()
    for t in range(300):
        env.render()
        print(observation)
        feedback, thbot = env.state
        graph.append(feedback)
        print(feedback)
        Ctl.update(feedback)
        action = [Ctl.output]
        print(action)
        print(Ctl.PTerm, Ctl.ITerm,Ctl.DTerm)
        observation, reward, done, info = env.step(action)
    plt.plot(graph[::10], "^-")
    graph = []
plt.title("PID performance")
string = "../result/"+str(time.time())+"PIDgraph.png"
plt.savefig(string)
env.close()