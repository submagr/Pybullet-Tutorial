import pybullet as p
import pybullet_data
import numpy as np
from matplotlib import pyplot as plt
from utils import step_sim
from pathlib import Path

image_size = (512, 512)
focal_length = 1000
z_near, z_far = 0.01, 10


def get_projection_matrix():
    fovh = (image_size[0] / 2) / focal_length
    fovh = 180 * np.arctan(fovh) * 2 / np.pi
    aspect_ratio = image_size[1] / image_size[0]
    return p.computeProjectionMatrixFOV(fovh, aspect_ratio, z_near, z_far)


def save_img(rgb, d, root_dir):
    plt.imsave(rgb, root_dir / "rgb.png")
    plt.imsave(d, root_dir / "d.png")


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
    projection_matrix = get_projection_matrix()
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[2, 2, 2],  # from
        cameraTargetPosition=[0, 0, 0],  # to
        cameraUpVector=[0, 0, 1],
    )
    img_arr = p.getCameraImage(
        width=image_size[1],
        height=image_size[0],
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
    )
    w = img_arr[0]
    h = img_arr[1]
    rgb = img_arr[2]
    rgb_arr = np.array(rgb, dtype=np.uint8).reshape([h, w, 4])
    rgb = rgb_arr[:, :, 0:3]

    d = img_arr[3]
    d = np.array(d).reshape([h, w])
    d = (2.0 * z_near * z_far) / (z_far + z_near - (2.0 * d - 1.0) * (z_far - z_near))

    save_img(rgb, d, Path("images/"))
