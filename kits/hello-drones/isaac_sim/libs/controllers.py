import numpy as np
#Safety Limits
MAX_XY_ACCEL = 2.0
MAX_Z_ACCEL = 3.0
POS_INTEGRAL_LIMIT = 2.0

class PIDController:
    def __init__(self, params):
        self.name = params["name"]
        self.kp_pos = params["kp_pos"]
        self.kd_pos = params["kd_pos"]
        self.ki_pos = params.get("ki_pos", 0.0)
        self.kp_att = params["kp_att"]
        self.kd_att = params["kd_att"]
        self.pos_integral = np.zeros(3)
        
    def reset_integrals(self):
        self.pos_integral = np.zeros(3)
        
    def compute_position_control(self, pos_error, vel_error, dt):
        p_term = self.kp_pos * pos_error
        d_term = self.kd_pos * vel_error
        
        if self.ki_pos > 0:
            self.pos_integral += pos_error * dt
            limit = POS_INTEGRAL_LIMIT
            self.pos_integral = np.clip(self.pos_integral, -limit, limit)
            i_term = self.ki_pos * self.pos_integral
        else:
            i_term = np.zeros(3)
            
        desired_acc = p_term + d_term + i_term
        
        # SAFETY CLAMP
        acc_xy = desired_acc[:2]
        xy_mag = np.linalg.norm(acc_xy)
        if xy_mag > MAX_XY_ACCEL:
            scale = MAX_XY_ACCEL / xy_mag
            acc_xy = acc_xy * scale
            
        acc_z = np.clip(desired_acc[2], -MAX_Z_ACCEL, MAX_Z_ACCEL)
            
        return np.array([acc_xy[0], acc_xy[1], acc_z])
    
    def compute_attitude_control(self, att_error, ang_vel):
        return self.kp_att * att_error - self.kd_att * ang_vel