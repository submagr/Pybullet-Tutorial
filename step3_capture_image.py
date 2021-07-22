import pybullet as p
import pybullet_data
import numpy as np
from matplotlib import pyplot as plt
from utils import step_sim
from pathlib import Path

IMAGE_SIZE = (512, 512)
FOCAL_LENGTH = 1000


# def get_projection_matrix():
#     fovh = (IMAGE_SIZE[0] / 2) / FOCAL_LENGTH
#     fovh = 180 * np.arctan(fovh) * 2 / np.pi
#     aspect_ratio = IMAGE_SIZE[1] / IMAGE_SIZE[0]
#     return p.computeProjectionMatrixFOV(45.0, 1.0, Z_NEAR, z_far)


def save_img(rgb, d, root_dir):
    plt.imsave(root_dir / "rgb.png", rgb)
    plt.imsave(root_dir / "d.png", d, cmap="gray")


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    obj_id = p.loadURDF(
        fileName="urdf/duck/duck.urdf",
        basePosition=[0, 0, 1],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    )
    step_sim(1e2)  # Let the object settle down

    # Setup camera
    Z_NEAR, Z_FAR = 0.01, 10
    # - Camera intrinsics
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=45.0,  # Field of view
        aspect=1.0,  # Aspect ratio
        nearVal=Z_NEAR,  # Near plane
        farVal=Z_FAR,  # Far plane
    )
    # - Camera extrinsics
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[2, 2, 2],  # from
        cameraTargetPosition=[0, 0, 0],  # to
        cameraUpVector=[0, 0, 1],
    )
    img_arr = p.getCameraImage(
        width=IMAGE_SIZE[1],
        height=IMAGE_SIZE[0],
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
    )
    w = img_arr[0]
    h = img_arr[1]
    # Process RGB
    rgba_buffer = img_arr[2]
    rgb = np.array(rgba_buffer, dtype=np.uint8).reshape([h, w, 4])[:, :, :3]
    # Process Depth
    z_buffer = img_arr[3]
    z_buffer = np.array(z_buffer).reshape([h, w])
    d = (2.0 * Z_NEAR * Z_FAR) / (Z_FAR + Z_NEAR - (2.0 * z_buffer - 1.0) * (Z_FAR - Z_NEAR))
    save_img(rgb, d, Path("images/"))
