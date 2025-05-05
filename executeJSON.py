import json
from math import radians
from urbasic import ISCoin, Joint6D


class ISCoinTrajectoryExecutor:
    def __init__(self, host, gripper_opened_mm=40, trajectory_file="trajectory.json", acc=radians(5), speed=radians(5)):
        self.host = host
        self.gripper_opened_mm = gripper_opened_mm
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

    def set_speed(self, speed_rad_per_sec):
        self.speed = radians(speed_rad_per_sec)
        print(f"Speed set to {self.speed} rad/s")

    def set_acceleration(self, acc_rad_per_sec2):
        self.acc = radians(acc_rad_per_sec2)
        print(f"Acceleration set to {self.acc} rad/s²")

    def read_trajectory(self, trajectory_file=None):
        with open(trajectory_file, "r") as file:
            data = json.load(file)["modTraj"]
            return [i["positions"] for i in data]

    def execute_trajectory(self, trajectory=None):
        if not self.iscoin:
            raise RuntimeError("ISCoin not connected.")
        
        for point in trajectory:
            joint = Joint6D.createFromRadList(point)
            self.iscoin.robot_control.movej(joint, a=self.acc, v=self.speed)
        print("Trajectory executed successfully.")

    def go_to_point(self, joint_positions):
        """
        Moves the robot to a specific joint position.
        
        :param joint_positions: List of 6 joint angles in radians.
        """
        if not self.iscoin:
            raise RuntimeError("ISCoin not connected.")
        
        if len(joint_positions) != 6:
            raise ValueError("joint_positions must be a list of 6 elements.")
        
        joint = Joint6D.createFromRadList(joint_positions)
        self.iscoin.robot_control.movej(joint, a=self.acc, v=self.speed)
        print("Moved to specified joint position.")


if __name__ == "__main__":
    host_ip = "10.30.5.159"
    # host_ip = "10.30.5.158"
    executor = ISCoinTrajectoryExecutor(host=host_ip)
    executor.connect()
    executor.activate_gripper()
    trajectory = executor.read_trajectory("trajectory.json")
    executor.execute_trajectory(trajectory)
