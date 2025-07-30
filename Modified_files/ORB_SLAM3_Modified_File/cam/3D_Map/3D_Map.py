import numpy as np
import matplotlib.pyplot as plt

def load_trajectory(filename):
    traj = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith("#") or len(line.strip()) == 0:
                continue
            data = list(map(float, line.strip().split()))
            tx, ty, tz = data[1:4]
            traj.append([tx, ty, tz])
    return np.array(traj)

trajectory = load_trajectory("CameraTrajectory.txt")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label='Camera Path')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title("ORB-SLAM3 Camera Trajectory")
plt.show()
