import skfuzzy as sf
import time
import numpy as np
from math import pi, log

class Fuzzy_PID:
    
    def __init__(self, Pmax, Pmin, Imax, Imin, Dmax, Dmin):
        self.Kpmax = Pmax
        self.Kpmin = Pmin
        self.Kimax = Imax
        self.Kimin = Imin
        self.Kdmax = Dmax
        self.Kdmin = Dmin
        self.sample_time = 0.0
        self.current_time = time.time()
        self.last_time = self.current_time
        self.tfm = self.tfm_generator(-pi, pi)
        self.dtfm = self.tfm_generator(-8,8)
        self.re = self.rule()
        self.rde = self.re.T
        self.rie = self.rule_ki()
        self.a = self.rule_alpha()
        self.b = self.a.T
        self.clear()
    
    def tfm_generator(self, xmin, xmax):
        x = (xmax - xmin)/2

        NB = np.array([xmin, xmin, xmin+1/3*x], dtype = np.float)
        NM = np.array([xmin, xmin+1/3*x, xmin+2/3*x], dtype = np.float)
        NS = np.array([xmin+1/3*x, xmin+2/3*x, xmin+x], dtype = np.float)
        ZE = np.array([xmin+2/3*x, xmin+x, xmax - 2/3*x], dtype = np.float)
        PS = np.array([xmin+x, xmax-2/3*x, xmax-x/3], dtype = np.float)
        PM = np.array([xmax-2/3*x, xmax-x/3, xmax], dtype = np.float)
        PB = np.array([xmax - 1/3*x, xmax, xmax], dtype = np.float)

        return [NB, NM, NS, ZE, PS, PM, PB]
    
    def membership(self, x, tfm):
        x = np.array([x])
        return [sf.trimf(x, tfm[0]), sf.trimf(x, tfm[1]),sf.trimf(x, tfm[2]),\
            sf.trimf(x, tfm[3]),sf.trimf(x, tfm[4]),sf.trimf(x, tfm[5]),sf.trimf(x, tfm[6])]
    
    def rule(self):
        return np.matrix([[3,4,5,6,5,4,3],[2,3,4,5,4,3,2],[1,2,3,4,3,2,1],\
            [0,1,2,3,2,1,0],[1,2,3,4,3,2,1],[2,3,4,5,4,3,2],[3,4,5,6,5,4,3]])
    
    def rule_alpha(self):
        return np.matrix([[2,2,2,2,2,2,2],[3,3,2,2,2,3,3],[4,3,3,2,3,3,4],\
            [5,4,3,3,3,4,5],[4,3,3,2,3,3,4],[3,3,2,2,2,3,3],[2,2,2,2,2,2,2]])

    def rule_ki(self):
        return np.matrix([[0,0,0,0,0,0,0],[0,0,0,1,0,0,0],[0,0,2,2,2,0,0],\
            [0,2,4,2,4,2,0],[0,0,2,2,2,0,0],[0,0,0,1,0,0,0],[0,0,0,0,0,0,0]])

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 10.0
        self.output = 0.0

    def update_K(self, error, d_error):
        self.Kp = self.re[np.argmax(self.membership(error,self.tfm)),\
            np.argmax(self.membership(d_error, self.dtfm))]/6 *(self.Kpmax-self.Kpmin)+self.Kpmin
        self.Kd = self.rde[np.argmax(self.membership(error, self.tfm)),\
            np.argmax(self.membership(d_error, self.dtfm))]/6 *(self.Kdmax-self.Kdmin)+self.Kdmin
        self.alpha = self.a[np.argmax(self.membership(error, self.tfm)),\
            np.argmax(self.membership(d_error, self.dtfm))]
        self.Ki = self.rie[np.argmax(self.membership(error, self.tfm)),\
            np.argmax(self.membership(d_error, self.dtfm))]/4 *(self.Kimax - self.Kimin)+self.Kimin

    def update(self, feedback_value, speed):
        error = self.SetPoint - feedback_value
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        d_error = speed
        self.update_K(error, d_error)

        if (delta_time >= self.sample_time):
            pTerm = self.Kp * error
            if (pTerm < -self.windup_guard):
                self.PTerm = -self.windup_guard
            elif (pTerm > self.windup_guard):
                self.PTerm = self.windup_guard
            else:
                self.PTerm = pTerm
            self.ITerm += self.Ki * error * delta_time
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            if delta_time > 0:
                self.DTerm = self.Kd * delta_error / delta_time
            if (self.DTerm < -self.windup_guard):
                self.DTerm = -self.windup_guard
            elif (self.DTerm > self.windup_guard):
                self.DTerm = self.windup_guard
            self.last_time = self.current_time
            self.last_error = error

            Output = self.PTerm + (self.ITerm) + (self.DTerm)
            if Output > 15:
                self.output = 15
            elif Output < -15:
                self.output = -15
            else:
                self.output = Output
    
    def setKp(self, Pmax, Pmin):
        self.Kpmax = Pmax
        self.Kpmin = Pmin

    def setKd(self, Dmax, Dmin):
        self.Kdmax = Dmax
        self.Kdmin = Dmin
    
    def setKi(self, Imax, Imin):
        self.Kimax = Imax
        self.Kimin = Imin
    
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
    
    def setSetPoint(self, setpoint):
        self.SetPoint = setpoint