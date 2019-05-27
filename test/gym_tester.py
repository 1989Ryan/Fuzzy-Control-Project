import os
import sys
lib_path = os.path.abspath(os.path.join(sys.path[0], '..'))
sys.path.append(lib_path)
import gym
import matplotlib.pyplot as plt
from src.PID import *
import math

Ctl = PID()
Ctl.setKp(9.5)
Ctl.setKi(5.5)
Ctl.setKd(1.15)
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
    plt.title("PID performance")
    string = "p6k7d1.5graph" + str(i_episode) + ".png"
    plt.savefig(string)
    graph = []
env.close()