import time

class PID:

    def __init__(self, P = 0.2, I = 0, D = 0):
        self.Kp, self.Ki, self.Kd = P, I, D
        self.sample_time = 0.0
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
    
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 15.0

        self.output = 0.0

    def update(self, feedback_value):
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

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
            if Output > 20:
                self.output = 20
            elif Output < -20:
                self.output = -20
            else:
                self.output = Output
    
    def setKp(self, Proportional_gain):
        self.Kp = Proportional_gain
    
    def setKi(self, Integral_gain):
        self.Ki = Integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
    
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
    
    def setSetPoint(self, setpoint):
        self.SetPoint = setpoint