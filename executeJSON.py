import json
from math import radians
from urbasic import ISCoin, Joint6D


class ISCoinTrajectoryExecutor:
    def __init__(self, host, gripper_opened_mm=40, trajectory_file="trajectory.json", acc=radians(5), speed=radians(5)):
        self.host = host
        self.gripper_opened_mm = gripper_opened_mm
        self.trajectory_file = trajectory_file
        self.acc = acc
        self.speed = speed
        self.iscoin = None

    def connect(self):
        print("Connecting to ISCoin...")
        self.iscoin = ISCoin(host=self.host, opened_gripper_size_mm=self.gripper_opened_mm)
        print("Connected to ISCoin")

    def activate_gripper(self):
        if self.iscoin:
            self.iscoin.gripper.activate()
            print("Gripper activated")

    def open_gripper(self):
        if self.iscoin:
            self.iscoin.gripper.open()
            print("Gripper opened")
    
    def close_gripper(self):
        if self.iscoin:
            self.iscoin.gripper.close()
            print("Gripper closed")

    def read_trajectory(self):
        with open(self.trajectory_file, "r") as file:
            data = json.load(file)["modTraj"]
            return [i["positions"] for i in data]

    def execute_trajectory(self):
        if not self.iscoin:
            raise RuntimeError("ISCoin not connected.")
        
        trajectory = self.read_trajectory()
        for point in trajectory:
            joint = Joint6D.createFromRadList(point)
            self.iscoin.robot_control.movej(joint, a=self.acc, v=self.speed)
        print("Trajectory executed successfully.")


if __name__ == "__main__":
    host_ip = "10.30.5.159"
    # host_ip = "10.30.5.158"
    executor = ISCoinTrajectoryExecutor(host=host_ip, trajectory_file="trajectory.json")
    executor.connect()
    executor.activate_gripper()
    executor.execute_trajectory()
