import pybullet as p
import pybullet_data
from utils import step_sim



if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    step_sim(1e5)
