from scipy.spatial.transform import Rotation as R 
import numpy as np

quat = [[0, 0, np.sin(np.pi/4), np.cos(np.pi/4)], [0, 0, np.sin(-np.pi/4), np.cos(-np.pi/4)]]
r = R.from_quat(quat)
# r = R.from_rotvec([0, 0, np.pi/2])
print(r.as_dcm()[0])