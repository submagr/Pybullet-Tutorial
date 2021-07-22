import time
import pybullet as p
import pybullet_data
import numpy as np
from utils import step_sim


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    obj_id = p.loadURDF(
        fileName="urdf/pendulum/double_pendulum.urdf",
        basePosition=[0, 0, 2],
        baseOrientation=p.getQuaternionFromEuler([np.pi / 4, 0, 0]),
        useFixedBase=True,
    )
    for _ in range(int(1e5)):
        p.setJointMotorControl2(bodyIndex=obj_id, jointIndex=0, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
        p.stepSimulation()

        time.sleep(1 /240)
    step_sim(1e5)
