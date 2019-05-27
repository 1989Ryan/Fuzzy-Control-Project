import skfuzzy
import time
import os
import sys
lib_path = os.path.abspath(os.path.join(sys.path[0], '..'))
sys.path.append(lib_path)
import gym
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import LogNorm
from src.Fuzzy_PID import *
import numpy as np
import math

Ctl = Fuzzy_PID(10,7,4,2,1.15, 0.75)
Ctl.setKp(10,3)
Ctl.setKi(9,0)
Ctl.setKd(0.9,0.3)
Ctl.setSampleTime(0.05)
Ctl.setSetPoint(0.0)
graph = []
Graph = []
a = np.arange(-pi,pi,pi/100)
b = np.arange(-8,8,8/100)
for i in a:
	for j in b:
		Ctl.update(i,j)
		graph.append(Ctl.output)
	Graph.append(graph)
	graph = []
print(Graph)
plt.imshow(Graph, extent=(np.amin(a), np.amax(a), np.amax(b), np.amin(b)),
            cmap=cm.hot)
plt.colorbar()
plt.savefig('hot.png')#先存，再show
plt.show()

'''
tfm = Ctl.tfm_generator(-pi, pi)
dtfm = Ctl.tfm_generator(-8,8)
graph = []
ele = 2*8 / 100
count = -8
indexing = []
labels = ["NB", "NM","NS","ZE","PS","PM","PB"]
for i in range(7):
    for j in range(100):
        graph.append(Ctl.membership(count, dtfm)[i])
        count += ele
        indexing.append(count)
    plt.plot(indexing,graph, "-",label = labels[i])
    graph = []
    indexing = []
    count = -8
plt.title("Angle Speed Membership")
plt.legend(loc = 'upper right')
string = "../result/membership2.png"
plt.savefig(string)
'''
'''
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
'''