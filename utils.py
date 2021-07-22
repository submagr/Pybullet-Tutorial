import pybullet as p
from time import sleep


def step_sim(steps) -> None:
    for _ in range(int(steps)):
        p.stepSimulation()
        sleep(1 / 240)

