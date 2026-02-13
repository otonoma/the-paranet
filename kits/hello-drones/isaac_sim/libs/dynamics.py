import numpy as np

class QuadPhysicsEngine:
    """
    Handles aerodynamic calculations (Thrust, Torque, Drag) for the vehicle.
    """
    def __init__(self):
        self.mass = 1.5
        self.gravity = 9.81
        
        # Aerodynamic Constants
        self.k_lift = 8.54858e-6  # Force (N) per (rad/s)^2
        self.k_moment = 1e-6      # Torque (N*m) per (rad/s)^2
        
        # Motor directions: -1 = CW, 1 = CCW
        # Configuration: [Front-Right, Rear-Left, Front-Left, Rear-Right]
        self.motor_directions = np.array([-1, -1, 1, 1])
        
        self.max_rpm = 1100.0
        
        # Linear Drag Coefficients [X, Y, Z]
        self.drag_factors = np.array([0.50, 0.30, 0.0])

    def compute_aero_forces(self, rotor_speeds, body_velocity):
        """
        Calculates all forces acting on the drone based on current state.
        Returns:
            rotor_forces: array of 4 vertical forces (one per motor)
            yaw_torque: scalar total torque around Z axis
            drag_vector: array of 3 drag forces [x, y, z]
        """
        # Safety clamp for physical limits
        speeds = np.clip(rotor_speeds, 0, self.max_rpm)
        
        # Vectorized Thrust Calculation: F = k * w^2
        speed_sq = np.square(speeds)
        rotor_forces = self.k_lift * speed_sq
        
        # Yaw Torque Calculation: Torque = k_m * w^2 * direction
        moments = self.k_moment * speed_sq * self.motor_directions
        yaw_torque = np.sum(moments)
        
        # Body Drag Calculation: Drag = -C * v
        drag_vector = -self.drag_factors * body_velocity
        
        return rotor_forces, yaw_torque, drag_vector