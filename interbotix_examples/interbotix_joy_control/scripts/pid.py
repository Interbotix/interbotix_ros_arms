# PID controller object used to do velocity-based-position-control for each of the joints

class PID:
    def __init__(self, P=1.0, I=0.0, D=0.0, U_MAX=3.0, U_MIN=-3.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.u_max = U_MAX
        self.u_min = U_MIN
        self.clear()

    ### @brief Reset all errors to 0
    def clear(self):
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    ### @brief Compute desired velocity [rad/s] given a target position [rad]
    ### @param ref - desired position [rad]
    ### @param actual - actual position [rad] based on motor encoder
    def compute_control(self, ref, actual):
        error = ref - actual
        i_error = self.i_error + error
        self.d_error = error - self.p_error
        self.p_error = error

        u = self.Kp*self.p_error + self.Ki*i_error + self.Kd*self.d_error

        if (self.u_min < u and u < self.u_max):
            self.i_error = i_error
        elif (u < self.u_min):
            u = self.u_min
        elif (u > self.u_max):
            u = self.u_max
        return u
