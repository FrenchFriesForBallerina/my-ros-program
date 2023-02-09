# constants:
Kp = 0.05 
Ki = 0 
Kd = 0.01 
I = 0 # pane piirangud peale

class PID_Controller: 
    def __init__(self):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.I = I
