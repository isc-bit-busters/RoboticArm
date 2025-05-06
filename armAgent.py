from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, OneShotBehaviour
import os
import asyncio
import spade
import json
from executeJSON import ISCoinTrajectoryExecutor
import threading


from pipe import generate_joint_from_data, generate_joints_from_data


class ArmAgent(Agent):
    class ArmCommandBehaviour(CyclicBehaviour):
        async def on_start(self):
            print("ArmCommandBehaviour started", flush=True)
            self.iscoin_executor = None
            self.host_ip = None
            self.gripper_activated = False

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
                body = msg.body.strip() if msg.body is not None else ""

                try:
                    if msg_type == "set_host_ip":
                        self.host_ip = body
                        print(f"Host IP set to {self.host_ip}", flush=True)
                        send_log_message(
                            body=f"Host IP set to {self.host_ip}",
                            sender_id="armClient",
                            msg_type="arm_log"
                        )
                        self.connect_executor()
                        send_log_message(
                            body="ISCoin executor connected",
                            sender_id="armClient", 
                            msg_type="arm_log"
                        )
                    elif self.iscoin_executor is None:
                        print("ISCoin executor not connected. Please set the host IP first.", flush=True)
                        send_log_message(
                            body="ISCoin executor not connected. Please set the host IP first.",
                            sender_id="armClient",
                            msg_type="arm_log"
                        )
                    elif msg_type == "trajectory":
                        data = json.loads(body)
                        trajectory = generate_joints_from_data(data)
                        self.iscoin_executor.execute_trajectory(trajectory)
                        print(f"Executed trajectory with {len(data)} points", flush=True)
                        send_log_message(
                            body=f"Executed trajectory with {len(data)} points",
                            sender_id="armClient",
                            msg_type="arm_log"
                        )
                    elif msg_type == "point": 
                        data = json.loads(body)
                        joint_positions = generate_joint_from_data(data)
                        self.iscoin_executor.go_to_point(joint_positions)
                        print(f"Moved to point: {joint_positions}", flush=True)
                        send_log_message(
                            body=f"Moved to point: {joint_positions}",
                            sender_id="armClient",
                            msg_type="arm_log"
                        )
                    elif msg_type == "activate_gripper":
                        if not self.gripper_activated:
                            self.iscoin_executor.activate_gripper()
                            self.gripper_activated = True
                            print("Gripper activated", flush=True)
                            send_log_message(
                                body="Gripper activated",
                                sender_id="armClient",
                                msg_type="arm_log"
                            )
                        else:
                            print("Gripper already activated", flush=True)
                            send_log_message(
                                body="Gripper already activated",
                                sender_id="armClient",
                                msg_type="arm_log"
                            )
                    elif msg_type == "open_gripper":
                        if self.gripper_activated:
                            self.iscoin_executor.open_gripper()
                            print("Gripper opened", flush=True)
                            send_log_message(
                                body="Gripper opened",
                                sender_id="armClient",
                                msg_type="arm_log"
                            )
                        else:
                            print("⚠️ Cannot open gripper: activate_gripper must be called first.", flush=True)
                            send_log_message(
                                body="⚠️ Cannot open gripper: activate_gripper must be called first.",
                                sender_id="armClient",
                                msg_type="arm_log"
                            )

                    elif msg_type == "close_gripper":
                        if self.gripper_activated:
                            self.iscoin_executor.close_gripper()
                            print("Gripper closed", flush=True)
                            send_log_message(
                                body="Gripper closed",
                                sender_id="armClient",
                                msg_type="arm_log"
                            )
                        else:
                            print("⚠️ Cannot close gripper: activate_gripper must be called first.", flush=True)
                            send_log_message(
                                body="⚠️ Cannot close gripper: activate_gripper must be called first.",
                                sender_id="armClient",
                                msg_type="arm_log"
                            )
                    elif msg_type == "set_speed":
                        speed = float(body)
                        self.iscoin_executor.set_speed(speed)
                        print(f"Speed set to {speed} rad/s", flush=True)
                        send_log_message(
                            body=f"Speed set to {speed} rad/s",
                            sender_id="armClient",
                            msg_type="arm_log"
                        )
                    elif msg_type == "set_acceleration":
                        acceleration = float(body)
                        self.iscoin_executor.set_acceleration(acceleration)
                        print(f"Acceleration set to {acceleration} rad/s²", flush=True)
                        send_log_message(
                            body=f"Acceleration set to {acceleration} rad/s²",
                            sender_id="armClient",
                            msg_type="arm_log"
                        )
                    else:
                        print(f"Unknown command type: {msg_type}", flush=True)
                        send_log_message(
                            body=f"Unknown command type: {msg_type}",
                            sender_id="armClient",
                            msg_type="arm_log"
                        )
                except Exception as e:
                    print(f"Error processing command: {e}", flush=True)
                    send_log_message(
                        body=f"Error processing command: {e}",
                        sender_id="armClient",
                        msg_type="arm_log"
                    )
            else:
                print(f"Did not receive any message after {timeout} seconds", flush=True)

        async def on_end(self):
            await self.agent.stop()

    async def setup(self):
        print("ArmAgent started setup", flush=True)
        b = self.ArmCommandBehaviour()
        self.add_behaviour(b)

class SenderAgent(Agent):
    class SendBehaviour(OneShotBehaviour):
        def __init__(self, to_jid, message, robot_id, msg_type):
            super().__init__()
            self.to_jid = to_jid
            self.message = message
            self.robot_id = robot_id
            self.msg_type = msg_type

        async def run(self):
            from spade.message import Message
            msg = Message(to=self.to_jid, body=self.message)
            msg.set_metadata("robot_id", self.robot_id)
            msg.set_metadata("type", self.msg_type)
            await self.send(msg)
            await asyncio.sleep(1)
            await self.agent.stop()

def send_log_message(body, sender_id="armClient", robot_id="armClient", msg_type="arm_log"):
    async def task():
        password = os.getenv("XMPP_PASSWORD", "plsnohack")
        agent = SenderAgent(f"{sender_id}@prosody", password)
        await agent.start(auto_register=True)
        to_jid = "receiverClient@prosody"
        agent.add_behaviour(agent.SendBehaviour(to_jid, body, robot_id, msg_type))
        await asyncio.sleep(3)
        await agent.stop()
    threading.Thread(target=lambda: asyncio.run(task()), daemon=True).start()

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