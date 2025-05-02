import numpy as np
import json
from scipy.spatial.transform import Rotation

# Mock imports assuming they exist in your environment
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics

execute = False

def norm_to_quat(normal):
    if np.allclose(normal, [0, 0, 0]):
        raise ValueError("Cannot normalize a zero vector (normal is [0, 0, 0])")

    normal = normal / np.linalg.norm(normal)

    if np.allclose(normal, [0, 0, 1]):
        quat = (0, 5.06e-4, 0, 9.9e-1)
        return quat / np.linalg.norm(quat)
    elif np.allclose(normal, [0, 0, -1]):
        quat = (0, 9.9e-1, 0, 5.06e-4)
        return quat / np.linalg.norm(quat)

    rotation_axis = np.cross((0, 0, 1), normal)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    cos_angle = np.dot([0, 0, 1], normal)
    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))

    r = Rotation.from_rotvec(angle * rotation_axis)
    quat = r.as_quat()

    if np.isnan(quat).any():
        raise ValueError("Quaternion contains NaNs")

    close_to_pos_1 = np.isclose(quat, 1)
    close_to_min_1 = np.isclose(quat, -1)
    quat[close_to_pos_1] = 9.9e-1
    quat[close_to_min_1] = -9.9e-1

    assert not np.isclose(quat, 1).any()
    assert not np.isclose(quat, -1).any()

    return quat

def generate_trajectory_file(data, filename):
    modTraj = []
    time_step = 1_000_000_000
    time = 4_000_000_000

    for arr in data:
        positions = [round(float(x), 4) if abs(x) >= 1e-4 else 0.0 for x in arr]
        velocities = [0.0] * 6
        ns_time = time % 1_000_000_000
        s_time = int((time - ns_time)/1_000_000_000)
        modTraj.append({
            "positions": positions,
            "velocities": velocities,
            "time_from_start": [s_time, ns_time],
        })
        time += time_step

    with open(filename, "w") as f:
        json.dump({"modTraj": modTraj}, f, indent=4)

    print(f"Trajectory file '{filename}' generated successfully.")

def generateTrajectoryFromPoses(poses, filename="trajectory.json", graph=False, verbose=False):
    kine = URKinematics("ur3e_gripper")
    multi = MultiURKinematics(kine)

    print(f"Number of poses: {len(poses)}")
    print(f"poses: {poses}")

    angles = multi.inverse_optimal(poses)

    print(f"Number of angles: {len(angles.trajectory)}")
    print(f"angles: {angles.trajectory}")
    generate_trajectory_file(angles.trajectory, filename)

    if verbose:
        number_angles = len(angles.trajectory)
        number_points = len(poses)
        ratio = number_angles / number_points
        print(f"Number of poses: {number_points}, Number of angles: {number_angles}, Ratio: {ratio}")

def generate_joints_from_data(data_6d, output_file="trajectory.json"):
    poses = []
    for x, y, z, nx, ny, nz in data_6d:
        quat = norm_to_quat(np.array([nx, ny, nz]))
        pose = [x, y, z] + list(quat)
        poses.append(pose)

    generateTrajectoryFromPoses(poses, filename=output_file, graph=True, verbose=True)

# Example usage
if __name__ == "__main__":
    trajectory_file_name = "trajectory.json"
    data = [
        (0.3, 0.2, 0.22, 0, 0, -1),
        (0.34, 0.2, 0.22, 0, 0, -1),
        (0.3, 0.22, 0.22, 0, 0, -1),
        (0.31, 0.25, 0.22, 0, 0, -1),
    ]
    generate_joints_from_data(data, trajectory_file_name)

    if execute:
        from executeJSON import ISCoinTrajectoryExecutor
        host_ip = "10.30.5.159"
        # host_ip = "10.30.5.158"
        executor = ISCoinTrajectoryExecutor(host=host_ip, trajectory_file=trajectory_file_name)
        executor.connect()
        executor.execute_trajectory()
