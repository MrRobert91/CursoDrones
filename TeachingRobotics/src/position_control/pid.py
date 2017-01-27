#Clase PID




class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D


    def update(self, feedback_value):
        return (feedback_value * self.Kp)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain



pidx = PID()


vx = pidx.update(3)
print(vx)
