import skfuzzy
import time
import os
import sys
lib_path = os.path.abspath(os.path.join(sys.path[0], '..'))
sys.path.append(lib_path)
import gym
import matplotlib.pyplot as plt
from src.Fuzzy_PID import *
import math

Ctl = Fuzzy_PID(10,7,4,2,1.15, 0.75)
Ctl.setKp(10,3)
Ctl.setKi(9,0)
Ctl.setKd(0.9,0.3)
Ctl.setSampleTime(0.05)
Ctl.setSetPoint(1.1)
graph = []

env = gym.make('Pendulum-v0')
for i_episode in range(10):
    observation = env.reset()
    Ctl.clear()
    for t in range(300):
        env.render()
        feedback, thbot = env.state
        graph.append(feedback)
        Ctl.update(feedback, thbot)
        action = [Ctl.output]
        print(action)
        print(Ctl.PTerm, Ctl.ITerm,Ctl.DTerm)
        observation, reward, done, info = env.step(action)
    plt.plot(graph[::10], "^-")
    graph = []
plt.title("Fuzzy PID performance")
string = "../result/"+str(time.time())+"Fuzzy_graph.png"
plt.savefig(string)
env.close() 