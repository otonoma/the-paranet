"""
Isaac Sim - Drone Swarm
"""

import os
import sys
import asyncio
import signal
import numpy as np
from isaacsim import SimulationApp

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
ISAAC_SIM_PATH = os.path.join(PROJECT_ROOT, "isaac_sim")
ASSETS_PATH = os.path.join(ISAAC_SIM_PATH, "assets", "iris_blue.usd")
BG_PATH = os.path.join(ISAAC_SIM_PATH, "assets", "bg.usd")

sys.path.append(PROJECT_ROOT)
sys.path.append(os.path.join(ISAAC_SIM_PATH, "libs"))

print(f"Project Root: {PROJECT_ROOT}")
print(f"Asset Path:   {ASSETS_PATH}")
print(f"BG Path:      {BG_PATH}")

######## Start app framework

config = {
    "headless": True,
    "width": 1920,
    "height": 1080,
}
simulation_app = SimulationApp(config)

import carb
import omni.usd
from pxr import UsdGeom, UsdShade, Sdf, Gf, UsdPhysics
from omni.isaac.core import World
from omni.isaac.core.utils import prims as prim_utils
from omni.isaac.core.utils.viewports import set_camera_view
import omni.isaac.core.utils.stage as stage_utils
from isaac_sim.drones.iris import IrisDrone
from actors import register_actors
from paranet_agent import connector, actor

from camera_recorder import (
    CameraLayoutGenerator,
    MultiCameraRecorder
)

NUM_DRONES = 50
GRID_SPACING = 1.5

CONTROLLER_PARAMS = {
    "name": "PD",
    "kp_pos": 3.0, 
    "kd_pos": 3.5, 
    "ki_pos": 0.2,
    "kp_att": 3.0, 
    "kd_att": 0.4
}

# Recording settings
RECORD_DURATION_MINUTES = 12  # Total simulation time
SIMULATION_FPS = 60  # Physics steps per second
OUTPUT_FPS = 30  # Output video FPS
SPEEDUP_RATIO = 1
FRAME_SKIP = int((SIMULATION_FPS / OUTPUT_FPS) * SPEEDUP_RATIO)  # = 2

# Camera settings - Formation area (where words appear)
FORMATION_FOCAL_POINT = (0.0, 0.0, 25.0) 
NUM_FORMATION_CAMERAS = 15  # Cameras watching word formations
NUM_FORMATION_TOP = 5  # Top-down cameras for formations
NUM_FORMATION_EAST = NUM_FORMATION_CAMERAS - NUM_FORMATION_TOP  # East-angled for formations

# Camera settings - Initial grid (where drones start)
GRID_FOCAL_POINT = (0, 12.5, 0.5)  # Center of initial drone grid
NUM_GRID_CAMERAS = 3  # Cameras watching takeoff/landing

# Total cameras
NUM_CAMERAS_TOTAL = NUM_FORMATION_CAMERAS + NUM_GRID_CAMERAS


class DroneSwarmSim:
    def __init__(self):
        self.num_drones = NUM_DRONES
        self.drones = []
        self.assets_path = ASSETS_PATH
        self.controller_params = CONTROLLER_PARAMS
        self._world_settings = {
            "stage_units_in_meters": 1.0,
            "physics_dt": 1.0/SIMULATION_FPS,
            "rendering_dt": 1.0/SIMULATION_FPS,
        }
        self._physics_ready = False
        self.recorder = None
        self.recording_active = True
        self.simulation_frame = 0
        self.max_simulation_frames = RECORD_DURATION_MINUTES * 60 * SIMULATION_FPS
    
    def setup_background(self) -> None:
        """Setup background from USD file"""
        prim_utils.create_prim(
            "/World/Background",
            usd_path=BG_PATH
        )
        
        print("Background loaded!")
        
    def setup_scene(self) -> None:
        """Setup the scene with drones"""
        self.setup_background()
        
        prim_utils.create_prim("/World/drones", "Xform")
        
        print(f"\nSpawning {self.num_drones} Drones...")
        for i in range(self.num_drones):
            row = i // 3
            col = i % 3
            x_pos = (col * GRID_SPACING) - GRID_SPACING
            y_pos = (row * GRID_SPACING)
            
            prim_path = f"/World/drones/drone_{i}"
            
            drone = IrisDrone(
                prim_path=prim_path,
                position=[x_pos, y_pos, 0.5],
                usd_path=ASSETS_PATH,
                controller_params=CONTROLLER_PARAMS
            )
            self.drones.append(drone)
    
    def setup_cameras(self):
        """
        Setup custom camera configuration
        """
        cameras = []

        # GROUP 1: Formation Cameras (where words appear)
        
        # 10 east-angled cameras with varied distances and elevations
        cameras.extend(CameraLayoutGenerator.generate_varied_cameras(
            focal_point=FORMATION_FOCAL_POINT,
            num_cameras=NUM_FORMATION_EAST,
            distance_range=(10, 50),
            elevation_range=(10, 50),
            primary_direction='east',
            name_prefix="formation_east"
        ))
        
        # 5 top-down cameras at different heights
        cameras.extend(CameraLayoutGenerator.generate_top_cameras(
            focal_point=FORMATION_FOCAL_POINT,
            num_cameras=NUM_FORMATION_TOP,
            heights=[25, 35, 50, 75, 100],
            rotate_90=True,
            name_prefix="formation_top"
        ))
        
        # GROUP 2: Grid Cameras (where drones start)
        
        # 3 cameras watching the initial grid from the East
        cameras.extend(CameraLayoutGenerator.generate_varied_cameras(
            focal_point=GRID_FOCAL_POINT,
            num_cameras=NUM_GRID_CAMERAS,
            distance_range=(15, 30),
            elevation_range=(10, 40),
            primary_direction='east',
            name_prefix="grid_east"
        ))
        
        return cameras
    
    async def setup_post_load(self) -> None:
        """Setup after scene load"""
        self._physics_ready = False
        self._world.add_physics_callback("sim_step", callback_fn=self.on_physics_step)
        
        print("\n" + "="*70)
        print("Initializing Generic Multi-Camera Recorder - DUAL FOCAL POINTS")
        print("="*70)
        
        camera_configs = self.setup_cameras()
        
        self.recorder = MultiCameraRecorder(
            camera_configs=camera_configs,
            base_dir="./drone_recordings",
            output_fps=OUTPUT_FPS,
            resolution=(1920, 1080),
            target_duration_seconds=4 * 60
        )
        
        print("Camera recorder ready!")
        print("="*70 + "\n")
    
    async def setup_post_reset(self) -> None:
        """Setup after reset"""
        self._physics_ready = False
    
    def on_physics_step(self, step_size) -> None:
        """Called every physics step"""
        if self._physics_ready:
            for drone in self.drones:
                # Skip removed drones (None entries)
                if drone is None:
                    continue
                
                drone.update(step_size)
            
            # Capture frame for recording
            if self.recording_active and self.recorder is not None:
                if self.simulation_frame % FRAME_SKIP == 0:
                    success = self.recorder.capture_frame()
                    if not success:
                        print(f"\nRecording complete! All {NUM_CAMERAS_TOTAL} videos saved.")
                        self.recording_active = False
                
                self.simulation_frame += 1
                
                if self.simulation_frame % (60 * SIMULATION_FPS) == 0:
                    minutes_elapsed = self.simulation_frame / (60 * SIMULATION_FPS)
                    output_seconds = self.recorder.frames_captured / OUTPUT_FPS
                    print(f"Simulation: {minutes_elapsed:.0f}/{RECORD_DURATION_MINUTES} min | Output video: {output_seconds:.0f}s")
                
                # Stop after recording duration
                if self.simulation_frame >= self.max_simulation_frames:
                    print("\nReached target recording duration!")
                    self.recording_active = False
                    
        else:
            print("Physics Started")
            print("="*70)
            self._physics_ready = True
    
    def start_actors(self):
        """Register and deploy actors"""
        register_actors(self.drones, self)
        connector.start()
        actor.deploy("isaac")
        print("Drone actors registered")
    
    def cleanup(self):
        """Cleanup resources"""
        if self.recorder is not None:
            print("\nClosing recorder...")
            self.recorder.close()
            self.recorder = None


######## Signal handling for clean shutdown

drone_sim = None

def signal_handler(sig, frame):
    """Handle Ctrl+C detected! Shutting down gracefully..."""
    print("\n\nCtrl+C detected! Shutting down gracefully...")
    if drone_sim is not None:
        drone_sim.cleanup()
    if simulation_app is not None:
        simulation_app.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


######## Setup simulation

stage_utils.create_new_stage()
drone_sim = DroneSwarmSim()
drone_sim._world = World(**drone_sim._world_settings)

async def setup():
    """Async setup"""
    drone_sim.setup_scene()
    set_camera_view(eye=[25, 25, 30], target=[0, 0, 25]) 
    
    await drone_sim._world.reset_async()
    await drone_sim._world.pause_async()
    
    await drone_sim.setup_post_load()
    await drone_sim.setup_post_reset()
    
    # Start actors
    drone_sim.start_actors()
    
    await drone_sim._world.play_async()

asyncio.ensure_future(setup())

######## Simulation loop

print("\n" + "="*70)
print("Drone Swarm Simulation with Dual-Focal Multi-Camera Recording")
print("="*70 + "\n")

try:
    while simulation_app.is_running():
        drone_sim._world.step(render=True)
        
        # Stop simulation after recording is complete
        if not drone_sim.recording_active and drone_sim._physics_ready:
            print("\nRecording finished! You can continue simulation or press Ctrl+C to exit.")
            drone_sim._physics_ready = False
    
except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    # Cleanup
    drone_sim.cleanup()
    simulation_app.close()
    print("Simulation closed.")