import numpy as np
from scipy.spatial.transform import Rotation
import carb
import omni.kit.commands
from pxr import Gf, UsdGeom

# Isaac Sim version compatibility
try:
    from isaacsim.core.utils.prims import define_prim, get_prim_at_path
except ImportError:
    from omni.isaac.core.utils.prims import define_prim, get_prim_at_path

from omni.isaac.dynamic_control import _dynamic_control

# Local Imports
from isaac_sim.libs.controllers import PIDController
from isaac_sim.libs.dynamics import QuadPhysicsEngine

class IrisDrone:
    def __init__(self, prim_path, position, usd_path, controller_params):
        self.prim_path = prim_path
        self.spawn_position = np.array(position)
        
        # Set initial target to hover at spawn position + 4m altitude
        self.target_position = self.spawn_position + np.array([0.0, 0.0, 4.0])
        
        # Setup USD Prim
        prim = define_prim(prim_path, "Xform")
        prim.GetReferences().AddReference(usd_path)
        
        xform = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*position))
        omni.kit.commands.execute("TransformPrimCommand", path=prim_path, new_transform_matrix=xform)
        
        # Initialize Physics and Control
        self.physics = QuadPhysicsEngine()
        self.controller = PIDController(controller_params)
        self._dc = None
        
        # State
        self.position = np.array(position)
        self.velocity = np.zeros(3)
        self.attitude = np.array([0, 0, 0, 1])
        self.angular_velocity = np.zeros(3)
        self.position_error_history = []
        
        # Propeller rotation state
        self.rotor_angles = np.zeros(4)
        self.rotor_velocities = np.zeros(4)
        self.propeller_xforms = [None] * 4 
        self.propeller_paths = [None] * 4
        self.rotation_frame_counter = 0
        
    def initialize_dc(self):
        if self._dc is None:
            self._dc = _dynamic_control.acquire_dynamic_control_interface()
            
    def update_state(self):
        if self._dc is None: return False
        rb = self._dc.get_rigid_body(self.prim_path + "/body")
        if not rb or rb == 0: return False
        
        pose = self._dc.get_rigid_body_pose(rb)
        self.position = np.array([pose.p.x, pose.p.y, pose.p.z])
        self.attitude = np.array([pose.r.x, pose.r.y, pose.r.z, pose.r.w])
        self.velocity = np.array(self._dc.get_rigid_body_linear_velocity(rb))
        
        ang_vel_world = np.array(self._dc.get_rigid_body_angular_velocity(rb))
        self.angular_velocity = Rotation.from_quat(self.attitude).inv().apply(ang_vel_world)
        return True
    
    def compute_control(self, dt):
        pos_error = self.target_position - self.position
        vel_error = -self.velocity
        desired_acc = self.controller.compute_position_control(pos_error, vel_error, dt)
        
        # Gravity compensation
        gravity_vec = np.array([0, 0, self.physics.mass * self.physics.gravity])
        desired_force_world = (self.physics.mass * desired_acc) + gravity_vec
        
        force_magnitude = np.linalg.norm(desired_force_world)
        if force_magnitude < 0.01: force_magnitude = 1.0
        desired_z_body = desired_force_world / force_magnitude
        
        rot = Rotation.from_quat(self.attitude)
        current_z_body = rot.apply(np.array([0, 0, 1]))
        att_error = np.cross(current_z_body, desired_z_body)
        
        desired_torque = self.controller.compute_attitude_control(att_error, self.angular_velocity)
        return self.force_and_torques_to_velocities(force_magnitude, desired_torque)
    
    def force_and_torques_to_velocities(self, force, torque):
        """
        Mixer to convert desired force and torques into rotor angular velocities.
        """
        if self._dc is None: return [0]*4
        try:
            rb_body = self._dc.get_rigid_body(self.prim_path + "/body")
            rotors = [self._dc.get_rigid_body(self.prim_path + f"/rotor{i}") for i in range(4)]
            relative_poses = self._dc.get_relative_body_poses(rb_body, rotors)
            
            k_f = self.physics.k_lift
            k_m = self.physics.k_moment
            dirs = self.physics.motor_directions

            aloc_matrix = np.zeros((4, 4))
            aloc_matrix[0, :] = k_f
            for i in range(4):
                aloc_matrix[1, i] = relative_poses[i].p[1] * k_f  # Roll effect
                aloc_matrix[2, i] = -relative_poses[i].p[0] * k_f # Pitch effect
            
            aloc_matrix[3, :] = k_m * dirs
            
            # Solve for squared angular velocities
            squared_ang_vel = np.linalg.pinv(aloc_matrix) @ np.array([force, torque[0], torque[1], torque[2]])
            squared_ang_vel[squared_ang_vel < 0] = 0.0
            return np.sqrt(squared_ang_vel).tolist()
        except Exception:
            return [0]*4

    def find_propeller_path(self, rotor_index):
        if self.propeller_paths[rotor_index] is not None:
            return self.propeller_paths[rotor_index]
        
        for prop_name in ["iris_prop_ccw", "iris_prop_cw"]:
            prop_path = f"{self.prim_path}/rotor{rotor_index}/{prop_name}"
            prop_prim = get_prim_at_path(prop_path)
            if prop_prim and prop_prim.IsValid():
                self.propeller_paths[rotor_index] = prop_path
                return prop_path
        return None

    def rotate_propellers(self, dt):
        """Update visual rotation of propellers"""
        self.rotation_frame_counter += 1
        if self.rotation_frame_counter < 5:
            return
        
        self.rotation_frame_counter = 0
        effective_dt = dt * 5
        
        dirs = self.physics.motor_directions

        for i in range(4):
            prop_path = self.find_propeller_path(i)
            if prop_path is None:
                continue
            
            angular_velocity_rad_s = self.rotor_velocities[i] * 10.0 * dirs[i]
            self.rotor_angles[i] += angular_velocity_rad_s * effective_dt
            self.rotor_angles[i] = self.rotor_angles[i] % (2 * np.pi)
            
            try:
                prop_prim = get_prim_at_path(prop_path)
                if prop_prim and prop_prim.IsValid():
                    if self.propeller_xforms[i] is None:
                        self.propeller_xforms[i] = UsdGeom.Xformable(prop_prim)
                    
                    xformable = self.propeller_xforms[i]
                    xformable.ClearXformOpOrder()
                    
                    angle_degrees = np.degrees(self.rotor_angles[i])
                    rotation_op = xformable.AddRotateZOp()
                    rotation_op.Set(angle_degrees)
                            
            except Exception:
                pass

    def update(self, dt):
        """Update drone physics and control"""
        self.initialize_dc()
        if not self.update_state(): 
            return
        
        rotor_velocities = self.compute_control(dt)
        self.rotor_velocities = np.array(rotor_velocities)
        
        # Calculate Physics
        body_vel = Rotation.from_quat(self.attitude).inv().apply(self.velocity)
        thrusts, net_torque, drag_force = self.physics.compute_aero_forces(self.rotor_velocities, body_vel)
        
        # Apply to Simulation
        for i in range(4):
            self.apply_force([0.0, 0.0, thrusts[i]], f"/rotor{i}")
            
        self.apply_torque([0.0, 0.0, net_torque], "/body")
        self.apply_force(drag_force, "/body")
        
        self.rotate_propellers(dt)
        self.position_error_history.append(np.linalg.norm(self.target_position - self.position))

    def apply_force(self, force, body_part):
        rb = self._dc.get_rigid_body(self.prim_path + body_part)
        if rb: self._dc.apply_body_force(rb, carb._carb.Float3(force), carb._carb.Float3([0,0,0]), False)

    def apply_torque(self, torque, body_part):
        rb = self._dc.get_rigid_body(self.prim_path + body_part)
        if rb: self._dc.apply_body_torque(rb, carb._carb.Float3(torque), False)