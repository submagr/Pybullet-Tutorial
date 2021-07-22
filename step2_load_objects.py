import pybullet as p
import pybullet_data
import numpy as np
from utils import step_sim


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    obj_id = p.loadURDF(
        fileName = "urdf/duck/duck.urdf",
        basePosition = [0, 0, 2],
        baseOrientation = p.getQuaternionFromEuler([np.pi / 4, 0, 0]),
    )
    step_sim(1e5)

    