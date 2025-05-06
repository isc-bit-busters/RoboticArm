from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
import os
import asyncio
import spade
import json
from executeJSON import ISCoinTrajectoryExecutor

from pipe import generate_joint_from_data, generate_joints_from_data


class ArmAgent(Agent):
    class ArmCommandBehaviour(CyclicBehaviour):
        async def on_start(self):
            print("ArmCommandBehaviour started", flush=True)
            self.iscoin_executor = None
            self.host_ip = None

        def connect_executor(self):
            try:
                self.iscoin_executor = ISCoinTrajectoryExecutor(host=self.host_ip)
                self.iscoin_executor.connect()
                print(f"✅ Connected to ISCoin at {self.host_ip}", flush=True)
            except Exception as e:
                print(f"❌ Failed to connect to ISCoin: {e}", flush=True)
                self.iscoin_executor = None

        async def run(self):
            timeout = 10
            print("Waiting for command...", flush=True)
            msg = await self.receive(timeout=timeout)
            if msg:
                robot_id = msg.metadata.get("robot_id", "unknown")
                msg_type = msg.metadata.get("type", "unknown")
                print(f"Received message from {robot_id} of type {msg_type}", flush=True)
                body = msg.body.strip()

                try:
                    if msg_type == "set_host_ip":
                        self.host_ip = body
                        print(f"Host IP set to {self.host_ip}", flush=True)
                        self.connect_executor()
                    elif self.iscoin_executor is None:
                        print("ISCoin executor not connected. Please set the host IP first.", flush=True)
                    elif msg_type == "trajectory":
                        data = json.loads(body)
                        trajectory = generate_joints_from_data(data)
                        self.iscoin_executor.execute_trajectory(trajectory)
                        print(f"Executed trajectory with {len(data)} points", flush=True)
                    elif msg_type == "point": 
                        data = json.loads(body)
                        joint_positions = generate_joint_from_data(data)
                        self.iscoin_executor.go_to_point(joint_positions)
                        print(f"Moved to point: {joint_positions}", flush=True)
                    elif msg_type == "activate_gripper":
                        self.iscoin_executor.activate_gripper()
                        print("Gripper activated", flush=True)
                    elif msg_type == "open_gripper":
                        self.iscoin_executor.open_gripper()
                        print("Gripper opened", flush=True)
                    elif msg_type == "close_gripper":
                        self.iscoin_executor.close_gripper()
                        print("Gripper closed", flush=True)
                    elif msg_type == "set_speed":
                        speed = float(body)
                        self.iscoin_executor.set_speed(speed)
                        print(f"Speed set to {speed} rad/s", flush=True)
                    elif msg_type == "set_acceleration":
                        acceleration = float(body)
                        self.iscoin_executor.set_acceleration(acceleration)
                        print(f"Acceleration set to {acceleration} rad/s²", flush=True)
                    else:
                        print(f"Unknown command type: {msg_type}", flush=True)
                except Exception as e:
                    print(f"Error processing command: {e}", flush=True)
            else:
                print(f"Did not receive any message after {timeout} seconds", flush=True)

        async def on_end(self):
            await self.agent.stop()

    async def setup(self):
        print("ArmAgent started setup", flush=True)
        b = self.ArmCommandBehaviour()
        self.add_behaviour(b)


def start_agent():
    async def agent_task():
        xmpp_username = "armClient"
        xmpp_server = "prosody"
        xmpp_password = os.getenv("XMPP_PASSWORD", "plsnohack")
        try:
            arm_agent = ArmAgent(f"{xmpp_username}@{xmpp_server}", xmpp_password)
            await arm_agent.start(auto_register=True)
            print("ArmAgent started", flush=True)
            await spade.wait_until_finished(arm_agent)
        except spade.exception.XMPPError as e:
            print(f"XMPP Error: {e}", flush=True)

    asyncio.run(agent_task())

if __name__ == "__main__":
    start_agent()