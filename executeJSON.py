import json
from math import radians
import os
from urbasic import ISCoin, CameraSettings, FocusSettings, Joint6D


iscoin = ISCoin(host="10.30.5.159", opened_gripper_size_mm=40)
# iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)

print("Connecting to ISCoin...")


iscoin.gripper.activate()

print("Gripper activated")

# iscoin.gripper.open()
# print("Gripper opened")


def readJson(filename):
    with open(filename, "r") as file:
        data = json.load(file)["modTraj"]
        points = [i["positions"] for i in data]
    return points


traj = readJson("trajectory.json")

acc = radians(5)
speed = radians(5)

for p in traj:
    iscoin.robot_control.movej(Joint6D.createFromRadList(p), a=acc, v=speed)

print("Trajectory executed successfully.")
