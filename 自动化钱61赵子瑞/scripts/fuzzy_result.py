import skfuzzy
import time
import os
import sys
lib_path = os.path.abspath(os.path.join(sys.path[0], '..'))
sys.path.append(lib_path)
import gym
import matplotlib.pyplot as plt
from src.Fuzzy_PID import *
from src.PID import *
import math
import numpy as np

Ctl = Fuzzy_PID(10,7,4,2,1.15, 0.75)
Ctl.setKp(11,3)
Ctl.setKi(10,0)
Ctl.setKd(0.9,0.3)
Ctl.setSampleTime(0.05)
Ctl.setSetPoint(1.1)
graph = []
error = []
ISE = 0.0
IAE = 0.0
env = gym.make('Pendulum-v0')
observation = env.reset()
for i in range(500):
    env.render()
Ctl.clear()
for t in range(500):
    env.render()
    feedback, thbot = env.state
    graph.append(feedback)
    error.append(Ctl.SetPoint-feedback)
    ISE += (Ctl.SetPoint - feedback)*0.05
    IAE += (Ctl.SetPoint - feedback)**2*0.05
    Ctl.update(feedback, thbot)
    action = [Ctl.output+np.random.random()]
    print(action)
    print(Ctl.PTerm, Ctl.ITerm,Ctl.DTerm)
    observation, reward, done, info = env.step(action)
plt.plot(graph[::10], "^-", label = "fuzzy pid")
graph = []

Ctl = PID()
Ctl.setKp(7)
Ctl.setKi(3)
Ctl.setKd(0.6)
Ctl.setSampleTime(0.05)
graph = []

observation = env.reset()
for t in range(500):
    env.render()
    print(observation)
    feedback, thbot = env.state
    graph.append(feedback)
    print(feedback)
    Ctl.update(feedback)
    action = [Ctl.output+np.random.random()]
    print(action)
    print(Ctl.PTerm, Ctl.ITerm,Ctl.DTerm)
    observation, reward, done, info = env.step(action)
plt.plot(graph[::10], "^-",label = "classic PID")

graph = []
plt.title("PID performance_with_noise")
plt.legend(loc="upper right")
string = "../result/"+str(time.time())+"Fuzzy_graph.png"
plt.savefig(string)
env.close()
print(ISE)
print(IAE) 