import sys
import csv
from simple_stream import app_logger as AppLogger
from simple_stream import sensor_config as SensorConfig
from simple_stream import sensor_stream as SensorStream
from simple_stream import stream_csv_writer as StreamCsvWriter
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

class Opal:
    def __init__(self):
        self.stream = SensorStream()
        self.csv_writer = StreamCsvWriter()
        self.logger = AppLogger()
        self.window_size = 100
        self.update_rate = 100 # Hz
        self.dt = 1 / self.update_rate # seconds
        self.data = []
        self.X = []
        self.P = []
        self.n_sensors = None
        self.device_ids = None

    def start_streaming(self):
        self.stream.start()
        self.n_sensors = len(self.stream.device_ids)
        self.device_ids = self.stream.device_ids
        for device_id in self.device_ids:
            self.data.append(np.empty((0, 6)))                          # initialize list of empty arrays for data, 6 columns (x,y,z gyro and accel)
            self.X.append(np.empty((0, 4)))                             # initialize list of empty arrays for quaternion states 
            self.P.append(np.empty((4, 4, 0)))                          # initialize list of empty arrays for error covariance matrices

    def update_data(self):
        try:
            sensor_data = self.stream.get_next()                        # outputs data in order of device_ids
            self.logger.logger.debug("Received sensor data: {0}".format(sensor_data))
            self.csv_writer.write(sensor_data)
            self._append_data(sensor_data)
        except Exception as e:
            self.logger.logger.error("Could not retrieve sensor data. Error: {0}"
                .format(e))
            
    def calculate_joint_angle(self):
        pass
    
    def _append_data(self, sensor_data):
        for idx, device_data in enumerate(sensor_data):
            self.data[idx] = np.append(self.data[idx], device_data[2:7])   # only extract and store gyro and accel data 
            # need to limit array size to window_size

    def _EKF(self):
        """ Implementation of Extended Kalman Filter on IMU data (gyroscope and accelerometer). 
        Returns X (quaternion array (N, 4)) and P (error covariance array (4, 4, N)). Quaternions are expressed in scalar-first notation as (w, x, y, z).
        """
        for sensor in len(self.data):
            data = self.data[sensor]
            accl = data[:, 0:2]
            gyro = data[:, 3:5] 

            # Initialization
            n, m = data.size
            X_all = np.zeros((n, 4))                                    # quaternion state
            P_all = np.zeros((4, 4, n))                                 # error covariance matrix
            bias_w = np.array([0, 0, 0])                                # gyroscope bias   
            bias_a = np.array([0, 0, 0])                                # accelerometer bias
            noise_w = 0.0                                               # gyroscope error variance
            noise_a = 0.0                                               # accelerometer error variance
            gc = 9.81                                                   # gravity magnitude, m/s^2

            initX = np.array([1, 0, 0, 0])                              # can change to better initial prediction
            initP = np.eye(4) * 1e-4                                    # can change to better initial prediction
            X_all[1,:] = initX
            P_all[1,:] = initP
            x = initX
            P = initP

            for i in range(1, n):
                # ========= Predict -- gyroscope propagation ========= 
                w = gyro[i, :] - bias_w

                omega = np.array([[0, -w[0], -w[1], -w[2]], 
                                  [w[0], 0, w[2], -w[1]],
                                  [w[1], -w[2], 0, w[0]],
                                  [w[2], w[1], -w[0], 0]])              
                FD = np.eye(4) + self.dt * omega / 2                    # propagation dynamics

                C = np.array([[-x[1], -x[2], -x[3]],
                              [x[0], -x[3], x[2]],
                              [x[3], x[0], -x[1]],
                              [-x[2], x[1], x[0]]]) / 2
                Q = (self.dt ** 2) * C @ (noise_w * np.eye(3)) @ C      # process noise matrix

                # Propagate the state and covariance
                x = FD @ x
                x = x / np.norm(x)                                      # normalize
                P = FD @ P @ FD.T + Q 

                # ========= Measurement Update ========= 
                a = accl[i, :] - bias_a

                # Use non-linear equation to estimate prediction
                a_pred = gc * np.array([[2*x[1]*x[3] - x[0]*x[2]], 
                                        [2*x[2]*x[3] + x[0]*x[2]], 
                                        [x[0]**2 - x[1]**2 - x[2]**2 + x[3]**2]])
                
                H = 2 * gc * np.array([[-x[2], x[3], -x[0], x[1]],
                                       [x[1], x[0], x[3], x[2]],
                                       [x[0], -x[1], -x[2], x[3]]])     # measurement matrix

                # Measurement noise R is appropriate when the acceleration magnitude is approximately equal to gravity
                check = abs(np.norm(a) - gc)
                epsilon = 0.05                                          # tolerance

                if check < epsilon:
                    R = np.eye(3) * noise_a
                else:
                    R = np.eye(3) * 100

                # ========= Kalman Update ========= 
                K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
                x = x + K @ (a.T - a_pred)
                x = x / np.norm(x)                                      # normalize
                P = (np.eye(4) - K @ H) @ P

                #  ========= Append to variables ========= 
                X_all[i, :] = x
                P_all[:, :, i] = P

            # ========= Update X and P variables in Opal class =========
            self.X[sensor] = X_all
            self.P[sensor] = P_all

    def _quat2euler(quat_mat):
        """
        Convert quaternion matrix into euler angles.
        
        Input
        ------
        quat_mat: (N, 4) array of quaternions where N is the number of observations and the quaternions are expressed in scalar-first notation as (w, x, y, z)

        Output
        ------
        eul_mat: (N, 3) array of euler angles where N is the number of observations and the euler angles are expressed as (X, Y, Z) rotations or (roll, pitch, yaw)
        """
        w = quat_mat[:,0]
        x = quat_mat[:,1]
        y = quat_mat[:,2]
        z = quat_mat[:,3]
        
        t0 = 2.0 * (w*x + y*z)
        t1 = 1.0 - 2.0 * (x*x + y*y)
        rot_x = np.arctan2(t0, t1)

        t2 = 2.0 * (w*y - z*x)
        t2 = 1.0 if t2 > 1.0 else t2                                    # enforce max and min values of 1.0 and -1.0
        t2 = -1.0 if t2 < 1.0 else t2
        rot_y = np.arctan2(t2)

        t3 = 2.0 * (w*z + x*y)
        t4 = 1.0 - 2.0 * (y*y + z*z)
        rot_z = np.arctan2(t3, t4)

        eul_mat = np.hstack(rot_x, rot_y, rot_z)
        return eul_mat
    
    def _rad2deg(rad):
        """
        Convert from radians to degrees.
        """
        return rad * 180.0 / np.pi
    
    def _quat2rot(quat):
        """ 
        Convert a quaternion (1, 4) to rotation matrix (3, 3). Input quaternions are expressed in scalar-first notation as (w, x, y, z).
        """
        quat_wlast = np.hstack([quat[1:], quat[0]])                     # convert to scalar-last notation for scipy Rotation function
        r = R.from_quat(quat_wlast)
        return r.as_matrix()
    
    def _angular_vel_wrt_frameA(R_A, R_B, gyro_A, gyro_B):
        """ 
        Finds the relative angular velocity with respect to IMU frame A. Rotation matrices are (3, 3) arrays and gyroscope measurements are (1, 3) arrays.
        Returns (1, 3) array for rotation about (x, y, z). 
        """
        R_BA = R_A @ R_B.T
        w_rel_frameA = R_BA @ gyro_B.T - gyro_A.T
        return np.reshape(w_rel_frameA, (1, 3))

    def _pc_axis(w_rel_frame_array):
        """ 
        Used to find anatomical frame axis using (N, 3) array of relative angular velocities with respect to a specific IMU frame (i.e. thigh IMU frame). 
        Returns the first principal component of the data (encaptures highest variability) as the principal axis. 
        """
        pca = PCA(n_components=3)
        pca.fit(w_rel_frame_array)
        axis = pca.components_[0]

        # Note: the estimated axis can converge to align with z or -z axis. Check if the axis is aligned with thigh z frame.
        # If the angle between the thigh x and estimated axis is less than 100, then it converged to align with +x instead of -x axis.
        cos_th = np.dot(axis, [1, 0, 0]) / (np.norm(axis) * np.norm([1, 0, 0]))
        ang = np.arccos(cos_th) * 180 / np.pi
        if ang < 100:
            # axis converged to negative of the desired axis
            axis = -1 * axis
        
        return axis
 
    def _calc_rot_from_axis(axis):
        """
        Use estimated (1, 3) axis to find appropriate (3, 3) rotation matrices. Assumes axis aligns with z-axis. 
        """
        z_T = axis
        y_init = [0, 1, 0]
        x_T = np.cross(y_init, z_T)
        x_T = x_T / np.norm(x_T)
        y_T = np.cross(z_T, x_T)
        
        return np.hstack(x_T, y_T, z_T)
    
    def _rot_AA_2_AB(R_AA, R_AB, R_A, R_B):
        """
        Rotation from anatomical frame AA to anatomical frame AB. 
        
        Inputs
        -------
        R_AA: rotation from IMU frame A to anatomical frame AA
        R_AB: rotation from IMU frame B to anatomical frame AB
        R_A: rotation from world frame to IMU frame A
        R_B: rotation from world frame to IMU frame B

        Outputs
        -------
        R_AA_AB: rotation from anatomical frame AA to anatomical frame AB

        """
        R_AA_AB = R_AA.T @ R_A @ R_B.T @ R_AB
        return R_AA_AB
    
    def _ang_from_rot(R_AA_AB):
        """
        Calculates Euler angles from rotation matrix. Expressed as (rot_Z, rot_X, rot_Y) or (yaw, pitch, roll).
        """
        r = R.from_matrix(R_AA_AB)
        return r.as_euler('zxy', degrees=True)