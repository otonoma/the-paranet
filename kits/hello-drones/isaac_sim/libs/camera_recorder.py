"""
Generic Multi-Camera Recorder for Isaac Sim
Automatically generates camera positions around a focal point
Supports varied angles, distances, and top-down views
"""

import os
import datetime
import cv2
import numpy as np
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass

try:
    import omni.replicator.core as rep
except ImportError:
    rep = None
    print("Warning: omni.replicator not available")


@dataclass
class CameraConfig:
    """Configuration for a single camera"""
    name: str
    position: Tuple[float, float, float]
    look_at: Tuple[float, float, float]
    rotation: Optional[Tuple[float, float, float]] = None  # (roll, pitch, yaw) in degrees


class CameraLayoutGenerator:
    """
    Generates camera positions around a focal point.
    Creates varied angles, distances, and perspectives.
    """
    
    @staticmethod
    def spherical_to_cartesian(radius: float, theta: float, phi: float, 
                               center: Tuple[float, float, float]) -> Tuple[float, float, float]:
        theta_rad = np.deg2rad(theta)
        phi_rad = np.deg2rad(phi)
        
        x = center[0] + radius * np.sin(phi_rad) * np.cos(theta_rad)
        y = center[1] + radius * np.sin(phi_rad) * np.sin(theta_rad)
        z = center[2] + radius * np.cos(phi_rad)
        
        return (float(x), float(y), float(z))
    
    @staticmethod
    def generate_varied_cameras(focal_point: Tuple[float, float, float],
                               num_cameras: int,
                               distance_range: Tuple[float, float] = (10, 50),
                               elevation_range: Tuple[float, float] = (15, 75),
                               primary_direction: Optional[str] = None,
                               name_prefix: str = "cam") -> List[CameraConfig]:
        """
        Generate cameras with varied distances and angles around a focal point.
        """
        cameras = []
        
        direction_map = {
            'north': 90,    # +Y
            'south': 270,   # -Y
            'east': 0,      # +X
            'west': 180,    # -X
            None: None
        }
        
        primary_theta = direction_map.get(primary_direction)
        
        for i in range(num_cameras):
            t = i / max(num_cameras - 1, 1)
            radius = distance_range[0] + t * (distance_range[1] - distance_range[0])
            phi = 90 - (elevation_range[0] + np.random.uniform(0, 1) * (elevation_range[1] - elevation_range[0]))
            
            if primary_theta is not None:
                spread = 45
                theta = primary_theta + np.random.uniform(-spread, spread)
            else:
                theta = np.random.uniform(0, 360)
            
            position = CameraLayoutGenerator.spherical_to_cartesian(
                radius, theta, phi, focal_point
            )
            
            cameras.append(CameraConfig(
                name=f"{name_prefix}_{i:02d}",
                position=position,
                look_at=focal_point
            ))
        
        return cameras
    
    @staticmethod
    def generate_top_cameras(focal_point: Tuple[float, float, float],
                            num_cameras: int = 3,
                            heights: Optional[List[float]] = None,
                            rotate_90: bool = True,
                            name_prefix: str = "top") -> List[CameraConfig]:
        """
        Generate top-down cameras at different heights.
        """
        if heights is None:
            base_height = 30
            heights = [base_height * (1 + i) for i in range(num_cameras)]
        
        cameras = []
        rotation = (0, 0, 90) if rotate_90 else None
        
        for i, height in enumerate(heights[:num_cameras]):
            position = (focal_point[0], focal_point[1], focal_point[2] + height)
            
            cameras.append(CameraConfig(
                name=f"{name_prefix}_{i:02d}",
                position=position,
                look_at=focal_point,
                rotation=rotation
            ))
        
        return cameras


class MultiCameraRecorder:
    """
    Generic multi-camera recorder for Isaac Sim.
    """
    
    def __init__(self, 
                 camera_configs: List[CameraConfig],
                 base_dir: str = "./recordings",
                 output_fps: int = 30,
                 resolution: Tuple[int, int] = (1920, 1080),
                 target_duration_seconds: int = 240):
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join(base_dir, f"run_{timestamp}")
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.camera_configs = camera_configs
        self.output_fps = output_fps
        self.res = resolution
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        self.annotators = {}
        self.video_writers = {}
        self.frames_captured = 0
        self.total_frames_needed = target_duration_seconds * output_fps
        
        self._setup_cameras()
        self._open_video_writers()
        
        print(f"MultiCameraRecorder initialized: {len(self.camera_configs)} cameras")
    
    def _setup_cameras(self):
        for cfg in self.camera_configs:
            cam = rep.create.camera(
                position=cfg.position,
                look_at=cfg.look_at,
                name=f"cam_{cfg.name}"
            )
            
            if cfg.rotation is not None:
                try:
                    import omni.usd
                    from pxr import Gf, UsdGeom
                    
                    stage = omni.usd.get_context().get_stage()
                    camera_prim = stage.GetPrimAtPath(f"/Replicator/Camera_Xform/cam_{cfg.name}")
                    
                    if camera_prim.IsValid():
                        xform = UsdGeom.Xformable(camera_prim)
                        if cfg.rotation[2] != 0:
                            xform.AddRotateZOp().Set(cfg.rotation[2])
                        if cfg.rotation[1] != 0:
                            xform.AddRotateYOp().Set(cfg.rotation[1])
                        if cfg.rotation[0] != 0:
                            xform.AddRotateXOp().Set(cfg.rotation[0])
                except Exception as e:
                    print(f"Warning: Could not apply rotation to {cfg.name}: {e}")
            
            rp = rep.create.render_product(cam, self.res)
            rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb_annot.attach([rp])
            self.annotators[cfg.name] = rgb_annot
    
    def _open_video_writers(self):
        """Open video writers for all cameras"""
        for name in self.annotators.keys():
            path = os.path.join(self.output_dir, f"{name}.mp4")
            self.video_writers[name] = cv2.VideoWriter(path, self.fourcc, self.output_fps, self.res)
    
    def capture_frame(self) -> bool:
        """Capture a frame from all cameras"""
        if self.frames_captured >= self.total_frames_needed:
            return False
        
        for name, annot in self.annotators.items():
            data = annot.get_data()
            if data is not None and data.size > 0:
                img_bgr = cv2.cvtColor(data[:, :, :3], cv2.COLOR_RGB2BGR)
                self.video_writers[name].write(img_bgr)
        
        self.frames_captured += 1
        return True
    
    def close(self):
        """Close all video writers"""
        for name, writer in self.video_writers.items():
            if writer is not None:
                writer.release()
        print(f"Recording complete. Location: {self.output_dir}")