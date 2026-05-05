
import pybullet as p
import pybullet_data
import numpy as np
from typing import Optional, Tuple, Any

class RocketLoader:
    def __init__(self, client):
        self.client = client

    def load(self, urdf_path):
        return p.loadURDF(urdf_path)

class PyBulletSimulation:

    def __init__(self, gui: bool = True, debug: bool = False):
        self.debug = debug
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        self.rocket_id: Optional[int] = None
        self.ground_id: Optional[int] = None

    def setup_environment(self, urdf_path: str = "rocket.urdf"):
        from cfg import Simcfg

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(*Simcfg.gravity)
        self.ground_id = p.loadURDF("plane.urdf", useFixedBase=True)

        loader = RocketLoader(self.client)
        self.rocket_id = loader.load(urdf_path)

    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if self.rocket_id is None:
            raise RuntimeError("Ракета не загружена")

        position, orientation = p.getBasePositionAndOrientation(self.rocket_id)
        linear_vel, angular_vel = p.getBaseVelocity(self.rocket_id)

        return (
            np.array(position),
            np.array(orientation),
            np.array(linear_vel),
            np.array(angular_vel)
        )

    def apply_force(self, force: np.ndarray):
        if self.rocket_id is None:
            raise RuntimeError("Ракета не загружена")

        p.applyExternalForce(
            self.rocket_id,
            -1,
            force,
            [0, 0, 0],
            p.WORLD_FRAME
        )

    def step(self):
        p.stepSimulation()

    def dispose(self):
        p.disconnect()
