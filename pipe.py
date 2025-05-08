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

def generateTrajectoryFromPoses(poses, filename=None, verbose=False):
    kine = URKinematics("ur3e_gripper")
    multi = MultiURKinematics(kine)

    print(f"Number of poses: {len(poses)}")
    print(f"poses: {poses}")

    angles = multi.inverse_optimal(poses, logs=verbose)

    print(f"Number of angles: {len(angles.trajectory)}")
    print(f"angles: {angles.trajectory}")
    if filename is not None:
        generate_trajectory_file(angles.trajectory, filename)

    if verbose:
        number_angles = len(angles.trajectory)
        number_points = len(poses)
        ratio = number_angles / number_points
        print(f"Number of poses: {number_points}, Number of angles: {number_angles}, Ratio: {ratio}")

    return angles.trajectory


def generate_joint_from_data(data_6d, output_file=None):
    if len(data_6d) != 6:
        raise ValueError("data_6d must contain exactly 6 elements.")
    x, y, z, nx, ny, nz = data_6d
    quat = norm_to_quat(np.array([nx, ny, nz]))
    pose = [x, y, z] + list(quat)

    trajectory = generateTrajectoryFromPoses([pose], filename=output_file, verbose=True)
    return trajectory[0]

def generate_joints_from_data(data_6d, output_file=None):
    poses = []
    for x, y, z, nx, ny, nz in data_6d:
        quat = norm_to_quat(np.array([nx, ny, nz]))
        pose = [x, y, z] + list(quat)
        poses.append(pose)

    return generateTrajectoryFromPoses(poses, filename=output_file, verbose=True)

def generate_joint_chunks_from_groups(grouped_points, output_file=None, verbose=True):
    """Given multiple groups of 6D points, optimize them together and split result into chunks per group."""
    all_poses = []
    group_sizes = []

    for group in grouped_points:
        group_poses = []
        for x, y, z, nx, ny, nz in group:
            quat = norm_to_quat(np.array([nx, ny, nz]))
            pose = [x, y, z] + list(quat)
            group_poses.append(pose)
        all_poses.extend(group_poses)
        group_sizes.append(len(group_poses))

    full_trajectory = generateTrajectoryFromPoses(all_poses, filename=output_file, verbose=verbose)

    # Split the optimized trajectory into chunks matching the original input groups
    chunks = []
    start = 0
    for size in group_sizes:
        chunks.append(full_trajectory[start:start + size])
        start += size

    return chunks

# Example usage
if __name__ == "__main__":
    # execute = True
    # trajectory_file_name = "trajectory.json"
    # data = [
    #     (0.3, 0.2, 0.22, 0, 0, -1),
    #     (0.34, 0.2, 0.22, 0, 0, -1),
    #     (0.3, 0.22, 0.22, 0, 0, -1),
    #     (0.31, 0.25, 0.22, 0, 0, -1),
    # ]
    # poses = generate_joints_from_data(data)

    # single_point = (0.3, 0.2, 0.26, 0, 0, -1)
    # signle_pose = generate_joint_from_data(single_point)

    # if execute:
    #     from executeJSON import ISCoinTrajectoryExecutor
    #     host_ip = "10.30.5.159"
    #     # host_ip = "10.30.5.158"
    #     executor = ISCoinTrajectoryExecutor(host=host_ip)
    #     executor.connect()

    #     executor.execute_trajectory(poses)

    #     print(f"Executing trajectory of {len(data)} points finished.")

    #     executor.go_to_point(signle_pose)
    #     print(f"Executing single point finished.")


#  ----------------------------------------
    from executeJSON import ISCoinTrajectoryExecutor
    ip = "10.30.5.159"

    # execute = True
    execute = False

    if execute:
        executor = ISCoinTrajectoryExecutor(host=ip)
        executor.connect()
        executor.activate_gripper()
        executor.open_gripper()

    points = [
        (0.31, -0.1, 0.3, 0, 0, -1),
        (0.31, 0.31, 0.25, 0, 0, -1),
        (0.31, 0.31, 0.2, 0, 0, -1),
        (0.31, 0.31, 0.16, 0, 0, -1),
    ]
    poses = generate_joints_from_data(points)
    if execute:
        executor.execute_trajectory(poses)
        executor.close_gripper()

    points = [

        (0.31, 0.31, 0.16, 0, 0, -1),
        (0.31, 0.31, 0.17, 0, 0, -1),
        (0.31, 0.31, 0.26, 0, 0, -1),
        (0.31, 0.31, 0.30, 1, 0, -1),
        (0.31, 0.31, 0.40, 1, 0, -1),
        (0.31, 0.01, 0.35, 1, 0, -1),
    ]

    poses = generate_joints_from_data(points)
    if execute:
        executor.execute_trajectory(poses)
        executor.open_gripper()

#  -----------------------------------------

    from executeJSON import ISCoinTrajectoryExecutor
    ip = "10.30.5.159"
    execute = False  # Set to True to run on hardware

    if execute:
        executor = ISCoinTrajectoryExecutor(host=ip)
        executor.connect()
        executor.activate_gripper()
        executor.open_gripper()

    # Provide multiple groups of points
    groups = [
        # First movement group
        [
            (0.31, -0.1, 0.3, 0, 0, -1),
            (0.31, 0.31, 0.25, 0, 0, -1),
            (0.31, 0.31, 0.2, 0, 0, -1),
            (0.31, 0.31, 0.16, 0, 0, -1),
        ],
        # Gripper close and lift group
        [
            (0.31, 0.31, 0.17, 0, 0, -1),
            (0.31, 0.31, 0.26, 0, 0, -1),
            (0.31, 0.31, 0.30, 1, 0, -1),
            (0.31, 0.31, 0.40, 1, 0, -1),
            (0.31, 0.01, 0.35, 1, 0, -1),
        ],
    ]

    # Generate optimized chunks
    joint_chunks = generate_joint_chunks_from_groups(groups)

    for i, chunk in enumerate(joint_chunks):
        print(f"Executing chunk {i + 1}/{len(joint_chunks)} with {len(chunk)} points.")
        if execute:
            executor.execute_trajectory(chunk)
        if i == 0 and execute:
            executor.close_gripper()
        if i == 1 and execute:
            executor.open_gripper()