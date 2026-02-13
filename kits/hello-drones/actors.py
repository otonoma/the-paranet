"""
Drone Swarm Actor
"""

import numpy as np
from typing import List
from PIL import Image, ImageDraw, ImageFont
from sklearn.cluster import KMeans

from paranet_agent import actor
from paranet_agent.actor import BaseActor, Conversation
from omni.isaac.core.utils import prims as prim_utils

@actor.type
class TaskStatus:
    status: str
    message: str

@actor.type
class DronePosition:
    drone_id: int
    x: float
    y: float
    z: float

@actor.type
class DroneInfo:
    drone_id: int
    status: str
    message: str

@actor.type
class NumDrones:
    num_drones: int

@actor.type
class FormationResponse:
    success: bool
    message: str
    positions: List[DronePosition]

@actor.actor
class Isaac_DroneSwarm(BaseActor):
    """Actor for drone swarm control"""
    
    def __init__(self, drones, sim):
        super().__init__()
        self.drones = drones
        self.sim = sim
    
    @actor.skill(subject="drone_swarm", response=NumDrones)
    def get_num_drones(self, conv: Conversation) -> None:
        """Get the number of drones in the simulation"""
        num_drones = len([d for d in self.drones if d is not None])
        conv.send_response(NumDrones(num_drones=num_drones))
    
    @actor.skill(subject="drone_swarm", response=TaskStatus)
    def set_goal(self, drone_id: int, x: float, y: float, z: float, conv: Conversation) -> None:
        """Set goal position for a specific drone"""
        if drone_id < 0 or drone_id >= len(self.drones):
            conv.send_response(TaskStatus(
                status='error',
                message=f'Invalid drone ID: {drone_id}. Valid range: 0-{len(self.drones)-1}'
            ))
            return
        
        drone = self.drones[drone_id]
        
        if drone is None:
            conv.send_response(TaskStatus(
                status='error',
                message=f'Drone {drone_id} has been removed from simulation'
            ))
            return
        
        target = np.array([x, y, z])
        drone.target_position = target
        drone.controller.reset_integrals()
        
        conv.send_response(TaskStatus(
            status='success',
            message=f'Drone {drone_id} goal set to ({x:.2f}, {y:.2f}, {z:.2f})'
        ))
    
    @actor.skill(subject="drone_swarm", response=DronePosition)
    def get_position(self, drone_id: int, conv: Conversation) -> None:
        """Get current position of a drone"""
        if drone_id < 0 or drone_id >= len(self.drones):
            conv.send_response(DronePosition(
                drone_id=drone_id,
                x=0.0,
                y=0.0,
                z=0.0
            ))
            return
        
        drone = self.drones[drone_id]
        
        if drone is None:
            conv.send_response(DronePosition(
                drone_id=drone_id,
                x=0.0,
                y=0.0,
                z=0.0
            ))
            return
        
        conv.send_response(DronePosition(
            drone_id=drone_id,
            x=float(drone.position[0]),
            y=float(drone.position[1]),
            z=float(drone.position[2])
        ))
    
    @actor.skill(subject="drone_swarm", response=DroneInfo)
    def spawn_drone(self, x: float, y: float, z: float, conv: Conversation) -> None:
        """Spawn a new drone at the specified position"""
        try:
            # Import IrisDrone here to avoid circular imports
            from isaac_sim.drones.iris import IrisDrone
            
            drone_id = len(self.drones)
            prim_path = f"/World/drones/drone_{drone_id}"
            
            drone = IrisDrone(
                prim_path=prim_path,
                position=[x, y, z],
                usd_path=self.sim.assets_path,
                controller_params=self.sim.controller_params
            )
            
            self.drones.append(drone)
            
            conv.send_response(DroneInfo(
                drone_id=drone_id,
                status='success',
                message=f'Drone {drone_id} spawned at ({x:.2f}, {y:.2f}, {z:.2f})'
            ))
        except Exception as e:
            conv.send_response(DroneInfo(
                drone_id=-1,
                status='error',
                message=f'Failed to spawn drone: {str(e)}'
            ))
    
    @actor.skill(subject="drone_swarm", response=TaskStatus)
    def spawn_drones(self, num_drones: int, grid_spacing: float, start_height: float, conv: Conversation) -> None:
        """Spawn N drones in a 3-column grid formation"""
        try:
            from isaac_sim.drones.iris import IrisDrone
            
            spawned_ids = []
            
            for i in range(num_drones):
                # Calculate grid position (3 columns)
                row = i // 3
                col = i % 3
                
                # Calculate X and Y positions (same logic as initial spawn)
                x = (col * grid_spacing) - grid_spacing
                y = row * grid_spacing
                z = start_height
                
                # Create drone
                drone_id = len(self.drones)
                prim_path = f"/World/drones/drone_{drone_id}"
                
                drone = IrisDrone(
                    prim_path=prim_path,
                    position=[x, y, z],
                    usd_path=self.sim.assets_path,
                    controller_params=self.sim.controller_params
                )
                
                self.drones.append(drone)
                spawned_ids.append(drone_id)
            
            conv.send_response(TaskStatus(
                status='success',
                message=f'Successfully spawned {num_drones} drones (IDs: {spawned_ids[0]}-{spawned_ids[-1]})'
            ))
            
        except Exception as e:
            conv.send_response(TaskStatus(
                status='error',
                message=f'Failed to spawn drones: {str(e)}'
            ))
    
    @actor.skill(subject="drone_swarm", response=TaskStatus)
    def remove_drone(self, drone_id: int, conv: Conversation) -> None:
        """Remove a drone from the simulation"""
        if drone_id < 0 or drone_id >= len(self.drones):
            conv.send_response(TaskStatus(
                status='error',
                message=f'Invalid drone ID: {drone_id}. Valid range: 0-{len(self.drones)-1}'
            ))
            return
        
        try:
            # Get the drone's prim path
            drone = self.drones[drone_id]
            prim_path = drone.prim_path
            
            prim_utils.delete_prim(prim_path)
            
            # Mark as None instead of removing to keep indices consistent
            self.drones[drone_id] = None
            
            conv.send_response(TaskStatus(
                status='success',
                message=f'Drone {drone_id} removed from simulation'
            ))
        except Exception as e:
            conv.send_response(TaskStatus(
                status='error',
                message=f'Failed to remove drone {drone_id}: {str(e)}'
            ))

@actor.actor(name='word_formation')
class WordFormationActor(BaseActor):
    """Actor for calculating drone positions to form words"""
    
    @actor.skill(subject="word_formation", response=FormationResponse)
    def calculate_positions(self, word: str, num_drones: int, width_meters: float, height_z: float, conv: Conversation) -> None:
        """
        Calculate drone positions to form a word rotated 90 degrees.
        """
        try:
            # Calculate 2D positions
            positions_2d = self._get_drone_formation(word, num_drones, width_meters)
            
            drone_positions = []
            for i, pos in enumerate(positions_2d):
                drone_positions.append(DronePosition(
                    drone_id=i,
                    x=float(pos[0]),
                    y=float(pos[1]),
                    z=float(height_z)
                ))
            
            conv.send_response(FormationResponse(
                positions=drone_positions,
                success=True,
                message=f"Calculated {len(drone_positions)} sorted positions for word '{word}' (Rotated 90°)"
            ))
            
        except Exception as e:
            conv.send_response(FormationResponse(
                positions=[],
                success=False,
                message=f"Error calculating positions: {str(e)}"
            ))
    
    def _get_drone_formation(self, word, num_drones, target_width_meters=10.0):
        """
        Calculates 2D coordinates rotated 90 degrees.
        """
        
        # --- 1. Robust Font Loading ---
        font_paths = [
            "arial.ttf",
            "/Library/Fonts/Arial.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "C:\\Windows\\Fonts\\arial.ttf"
        ]
        
        font = None
        font_size = 100
        
        for path in font_paths:
            try:
                font = ImageFont.truetype(path, size=font_size)
                break
            except (IOError, OSError):
                continue
                
        if font is None:
            font = ImageFont.load_default()

        # --- 2. Dynamic Canvas Sizing ---
        dummy_draw = ImageDraw.Draw(Image.new('L', (1, 1)))
        bbox = dummy_draw.textbbox((0, 0), word, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]
        
        canvas_w = text_width + 40
        canvas_h = text_height + 40
        
        image = Image.new('L', (canvas_w, canvas_h), color=0)
        draw = ImageDraw.Draw(image)
        
        x_text = (canvas_w - text_width) // 2
        y_text = (canvas_h - text_height) // 2
        draw.text((x_text, y_text), word, font=font, fill=255)

        # --- 3. Extract Point Cloud & ROTATE 90 Degrees ---
        grid = np.array(image)
        rows, cols = np.where(grid > 0)
        
        new_x = rows
        new_y = cols
        
        points = np.column_stack((new_x, new_y))
        
        # --- 4. Safety Check ---
        if len(points) == 0:
            raise ValueError("Text generated no pixels.")
            
        if len(points) < num_drones:
            indices = np.random.choice(len(points), num_drones, replace=True)
            points = points[indices]
        
        # --- 5. Clustering (K-Means) ---
        kmeans = KMeans(n_clusters=num_drones, n_init=10, random_state=42)
        kmeans.fit(points)
        centers = kmeans.cluster_centers_
        
        # --- 6. SORTING ---
        # Sort by Y (vertical word direction) primarily.
        sort_order = np.lexsort((centers[:, 0], centers[:, 1]))
        centers = centers[sort_order]
        
        # --- 7. Scaling to Real World Meters ---
        min_x, max_x = centers[:, 0].min(), centers[:, 0].max()
        min_y, max_y = centers[:, 1].min(), centers[:, 1].max()
        
        # Scale based on the word's length
        height_pixel = max_y - min_y if max_y != min_y else 1.0
        scale_factor = target_width_meters / height_pixel
        
        # Center the formation around (0,0)
        centers[:, 0] = (centers[:, 0] - ((max_x + min_x) / 2)) * scale_factor
        centers[:, 1] = (centers[:, 1] - ((max_y + min_y) / 2)) * scale_factor
        
        return centers

def register_actors(drones, sim):
    """Register all drone actors"""
    # Register the main swarm controller (requires sim context)
    actor.register_actor(Isaac_DroneSwarm(drones, sim))
    
    # Register the word formation calculator (stateless)
    actor.register_actor(WordFormationActor())