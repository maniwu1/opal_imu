import sys
import csv
# sys.path.append('../')
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
        self.logger = AppLogger(__name__)
        self.update_rate = 100 # Hz
        self.dt = 1 / self.update_rate # seconds
        self.window_size = 5 * self.update_rate # 5 seconds of data
        self.data = []
        self.X = []
        self.P = []
        self.n_sensors = None
        self.device_ids = None
        self.device_labels = dict()
        self.IMU2anatomical_rot = dict()
        self.joint_angles = dict()

    def configure(self):
        SensorConfig.configure() # APDM sensors must be configured prior to streaming
        print("Remove the sensors from the docking station. Wait until the " +
            "sensors and the access point are all flashing green in unison " + 
            "before streaming.")

    def start_streaming(self):
        self.stream.start()
        self.n_sensors = len(self.stream.device_ids)
        self.device_ids = self.stream.device_ids
        for device_id in self.device_ids:
            self.data.append(np.empty((0, 6)))                          # initialize list of empty arrays for data, 6 columns (x,y,z gyro and accel)
            self.X.append(np.empty((0, 4)))                             # initialize list of empty arrays for quaternion states 
            self.P.append(np.empty((4, 4, 0)))                          # initialize list of empty arrays for error covariance matrices
            ## TODO: need to match device ids with device labels (back, thigh, shank, foot for left and right )
            ##       Initial thought is to use a json file to prestore device IDs and labels, assuming that they do not change. 

    def update_data(self):
        try:
            sensor_data = self.stream.get_next()                        # outputs data in order of device_ids
            self.logger.logger.debug("Received sensor data: {0}".format(sensor_data))
            self.csv_writer.write(sensor_data)
            self._append_data(sensor_data)
            self._EKF
            self._update_joint_angles
        except Exception as e:
            self.logger.logger.error("Could not retrieve sensor data. Error: {0}"
                .format(e))
            
    def calibrate_axes(self):
        """
        Called during calibration procedures to find rotation matrix from IMU frame to anatomical frame.
        """ 

        # Calculate torso, left hip, and right hip rotation matrices
        torso_idx = self.device_ids.index(self.device_labels['Torso'])
        lthigh_idx = self.device_ids.index(self.device_labels['L Thigh'])
        rthigh_idx = self.device_ids.index(self.device_labels['R Thigh'])

        axis_torso, axis_lthigh = self._calculate_joint_axis(self.X[torso_idx], self.X[lthigh_idx], 
                                                             self.data[torso_idx][:,0:3], self.data[lthigh_idx][:,0:3])
        axis_torso2, axis_rthigh = self._calculate_joint_axis(self.X[torso_idx], self.X[rthigh_idx], 
                                                             self.data[torso_idx][:,0:3], self.data[rthigh_idx][:,0:3])
        
        self.IMU2anatomical_rot['Torso'] = self._calc_rot_from_axis(axis_torso)
        self.IMU2anatomical_rot['L Thigh'] = self._calc_rot_from_axis(axis_lthigh)
        self.IMU2anatomical_rot['R Thigh'] = self._calc_rot_from_axis(axis_rthigh)

        # Calculate left shank and ankle rotation matrices
        lshank_idx = self.device_ids.index(self.device_labels['L Shank'])
        lfoot_idx = self.device_ids.index(self.device_labels['L Foot'])
        axis_lshank, axis_lfoot = self._calculate_joint_axis(self.X[lshank_idx], self.X[lfoot_idx], 
                                                             self.data[lshank_idx][:,0:3], self.data[lfoot_idx][:,0:3])
        self.IMU2anatomical_rot['L Shank'] = self._calc_rot_from_axis(axis_lshank)
        self.IMU2anatomical_rot['L Foot'] = self._calc_rot_from_axis(axis_lfoot)

        # Calculate right shank and ankle rotation matrices 
        rshank_idx = self.device_ids.index(self.device_labels['R Shank'])
        rfoot_idx = self.device_ids.index(self.device_labels['R Foot'])
        axis_rshank, axis_rfoot = self._calculate_joint_axis(self.X[rshank_idx], self.X[rfoot_idx], 
                                                             self.data[rshank_idx][:,0:3], self.data[rfoot_idx][:,0:3])
        self.IMU2anatomical_rot['R Shank'] = self._calc_rot_from_axis(axis_rshank)
        self.IMU2anatomical_rot['R Foot'] = self._calc_rot_from_axis(axis_rfoot)


    ################################# INTERNAL FUNCTIONS ####################################
    
    def _append_data(self, sensor_data):
        for idx, device_data in enumerate(sensor_data):
            self.data[idx] = np.append(self.data[idx], device_data[2:7])   # only extract and store gyro and accel data 
            # TODO: need to limit array size to window_size

    def _EKF(self):
        """ Implementation of Extended Kalman Filter on IMU data (gyroscope and accelerometer). 
        Returns X (quaternion array (N, 4)) and P (error covariance array (4, 4, N)). Quaternions are expressed in scalar-first notation as (w, x, y, z).
        """
        for sensor in len(self.data):
            data = self.data[sensor]
            accl = data[:, 0:3]
            gyro = data[:, 3:6] 

            # Initialization
            n, m = data.shape
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
            P_all[:,:,1] = initP
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
                Q = np.matmul(np.matmul((self.dt ** 2) * C, (noise_w * np.eye(3))), C.T)      # process noise matrix

                # Propagate the state and covariance
                x = np.matmul(FD, x)
                x = x / np.linalg.norm(x)                                      # normalize
                P = np.matmul(np.matmul(FD, P), FD.T) + Q 

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
                check = abs(np.linalg.norm(a) - gc)
                epsilon = 0.05                                          # tolerance

                if check < epsilon:
                    R = np.eye(3) * noise_a
                else:
                    R = np.eye(3) * 100

                # ========= Kalman Update ========= 
                K = np.matmul(np.matmul(P, H.T), np.linalg.inv(np.matmul(np.matmul(H, P), H.T) + R))
                x = x + np.matmul(K, (a.T - a_pred))
                x = x / np.linalg.norm(x)                                      # normalize
                P = np.matmul((np.eye(4) - np.matmul(K, H)), P)

                #  ========= Append to variables ========= 
                X_all[i, :] = x
                P_all[:, :, i] = P

            # ========= Update X and P variables in Opal class =========
            self.X[sensor] = X_all
            self.P[sensor] = P_all

    def _quat2euler(self, quat_mat, seq):
        """
        Convert quaternion matrix into euler angles.
        
        Input
        ------
        quat_mat: (N, 4) array of quaternions where N is the number of observations and the quaternions are expressed in scalar-first notation as (w, x, y, z)

        Output
        ------
        eul_mat: (N, 3) array of euler angles where N is the number of observations and the euler angles are expressed as (X, Y, Z) rotations or (roll, pitch, yaw)
        """
        # w = quat_mat[:,0]
        # x = quat_mat[:,1]
        # y = quat_mat[:,2]
        # z = quat_mat[:,3]
        
        # t0 = 2.0 * (w*x + y*z)
        # t1 = 1.0 - 2.0 * (x*x + y*y)
        # rot_x = np.arctan2(t0, t1)

        # t2 = 2.0 * (w*y - z*x)
        # t2 = 1.0 if t2 > 1.0 else t2                                    # enforce max and min values of 1.0 and -1.0
        # t2 = -1.0 if t2 < 1.0 else t2
        # rot_y = np.arctan2(t2)

        # t3 = 2.0 * (w*z + x*y)
        # t4 = 1.0 - 2.0 * (y*y + z*z)
        # rot_z = np.arctan2(t3, t4)

        # eul_mat = np.hstack((rot_x, rot_y, rot_z))
        # return eul_mat
        quat_wlast = np.hstack([quat_mat[:, 1:], quat_mat[:, 0].reshape(-1, 1)])
        r = R.from_quat(quat_wlast)
        return r.as_euler(seq, degrees=True)
    
    def _rad2deg(self, rad):
        """
        Convert from radians to degrees.
        """
        return rad * 180.0 / np.pi
    
    def _quat2rot(self, quat):
        """ 
        Convert a quaternion (N, 4) to rotation matrix (N, 3, 3). Input quaternions are expressed in scalar-first notation as (w, x, y, z).
        Inputs may be 2d or 3d arrays.
        """
        quat_wlast = np.hstack([quat[:, 1:], quat[:, 0].reshape(-1,1)])                   # convert to scalar-last notation for scipy Rotation function
        r = R.from_quat(quat_wlast)
        rot = r.as_dcm() # returns rotation matrix that is the transpose of Matlab output
        return rot.transpose((0,2,1))
    
    def _angular_vel_wrt_frameA(self, R_A, R_B, gyro_A, gyro_B):
        """ 
        Finds the relative angular velocity with respect to IMU frame A. Rotation matrices are (N, 3, 3) arrays and gyroscope measurements are (N, 3) arrays.
        Returns (N, 3) array for rotation about (x, y, z). 
        """
        N, a, b = R_A.shape
        gyro_A_ = gyro_A.reshape((N, 3, 1))
        gyro_B_ = gyro_B.reshape((N, 3, 1))

        R_BA = np.matmul(R_A, np.transpose(R_B, (0, 2, 1)))                           # transpose last two dimensions, results in (N, 3, 3)
        w_rel_frameA = np.matmul(R_BA, np.transpose(gyro_B_, (0, 2, 1))) - np.transpose(gyro_A_, (0, 2, 1))
        return w_rel_frameA.reshape((-1, 3))
    
    def _angular_vel_wrt_frameB(self, R_A, R_B, gyro_A, gyro_B):
        """ 
        Finds the relative angular velocity with respect to IMU frame B. Rotation matrices are (N, 3, 3) arrays and gyroscope measurements are (N, 3) arrays.
        Returns (N, 3) array for rotation about (x, y, z). 
        """
        N, a, b = R_A.shape
        gyro_A_ = gyro_A.reshape((N, 3, 1))
        gyro_B_ = gyro_B.reshape((N, 3, 1))

        R_AB = np.matmul(R_B, np.transpose(R_A, (0, 2, 1)))  
        w_rel_frameB = np.transpose(gyro_B_, (0, 2, 1)) - np.matmul(R_AB, np.transpose(gyro_A_, (0, 2, 1)))
        return w_rel_frameB.reshape((-1, 3))

    def _pc_axis(self, w_rel_frame_array):
        """ 
        Used to find anatomical frame axis using (N, 3) array of relative angular velocities with respect to a specific IMU frame (i.e. thigh IMU frame). 
        Returns the first principal component of the data (encaptures highest variability) as the principal axis. 
        """
        pca = PCA(n_components=3)
        pca.fit(w_rel_frame_array)
        axis = pca.components_[0]

        # Note: the estimated axis can converge to align with z or -z axis. Check if the axis is aligned with thigh z frame.
        # If the angle between the thigh x and estimated axis is less than 100, then it converged to align with +x instead of -x axis.
        cos_th = np.dot(axis, [1, 0, 0]) / (np.linalg.norm(axis) * np.linalg.norm([1, 0, 0]))
        ang = np.arccos(cos_th) * 180 / np.pi
        if ang < 100:
            # axis converged to negative of the desired axis
            axis = -1 * axis
        
        return axis
    
    def _calculate_joint_axis(self, X_A, X_B, gyro_A, gyro_B):
        R_A = self._quat2rot(X_A)
        R_B = self._quat2rot(X_B)
        w_rel_frameA = self._angular_vel_wrt_frameA(R_A, R_B, gyro_A, gyro_B)
        w_rel_frameB = self._angular_vel_wrt_frameB(R_A, R_B, gyro_A, gyro_B)

        axis_A = self._pc_axis(w_rel_frameA)
        axis_B = self._pc_axis(w_rel_frameB)
        return axis_A, axis_B
 
    def _calc_rot_from_axis(self, axis):
        """
        Use estimated (1, 3) axis to find appropriate (3, 3) rotation matrices. Assumes axis aligns with z-axis. 
        """
        z_T = axis
        y_init = [0, 1, 0]
        x_T = np.cross(y_init, z_T)
        x_T = x_T / np.linalg.norm(x_T)
        y_T = np.cross(z_T, x_T)
        
        return np.hstack(x_T, y_T, z_T)
    
    def _rot_AA_2_AB(self, R_AA, R_AB, R_A, R_B):
        """
        Rotation from anatomical frame AA to anatomical frame AB. 
        
        Inputs
        -------
        R_AA: (3, 3) rotation from IMU frame A to anatomical frame AA 
        R_AB: (3, 3) rotation from IMU frame B to anatomical frame AB
        R_A: (N, 3, 3) rotation from world frame to IMU frame A
        R_B: (N, 3, 3) rotation from world frame to IMU frame B

        Outputs
        -------
        R_AA_AB: rotation from anatomical frame AA to anatomical frame AB

        """
        R_AA_AB = np.matmul(np.matmul(np.matmul(R_AA.T, R_A), np.transpose(R_B, (0, 2, 1))), R_AB)
        return R_AA_AB
    
    def _ang_from_rot(self, R_AA_AB):
        """
        Calculates Euler angles from rotation matrix. Expressed as (rot_Z, rot_X, rot_Y) or (yaw, pitch, roll).
        Inputs may be 2d or 3d arrays. 
        """
        r = R.from_dcm(R_AA_AB)
        return r.as_euler('zxy', degrees=True)
    
    def _calculate_joint_angle(self, X_A, X_B, R_AA, R_AB):
        """ Calculates anatomical joint angle using quaternions from sensors A and B. Joint angles are calculated from anatomical frame A to anatomical frame B.
        """
        R_A = self._quat2rot(X_A)
        R_B = self._quat2rot(X_B)
        R_AA_AB = self._rot_AA_2_AB(R_AA, R_AB, R_A, R_B)
        
        joint_angle = self._ang_from_rot(R_AA_AB)
        return joint_angle

    def _update_joint_angles(self):
        """
        Updates joint angle estimates using links surrounding the joint. 
        """
        # Find device id indices from device labels to access data and IMU to anatomical rotation matrices 
        torso_idx = self.device_ids.index(self.device_labels['Torso'])
        lthigh_idx = self.device_ids.index(self.device_labels['L Thigh'])
        rthigh_idx = self.device_ids.index(self.device_labels['R Thigh'])
        lshank_idx = self.device_ids.index(self.device_labels['L Shank'])
        lfoot_idx = self.device_ids.index(self.device_labels['L Foot'])
        rshank_idx = self.device_ids.index(self.device_labels['R Shank'])
        rfoot_idx = self.device_ids.index(self.device_labels['R Foot'])

        # Hip angle estimates, saggital flexion/extension is stored in first dimension
        self.joint_angles['L Hip'] = self._calculate_joint_angle(self.X[torso_idx], self.X[lthigh_idx],
                                                                 self.IMU2anatomical_rot['Torso'], self.IMU2anatomical_rot['L Thigh'])
        self.joint_angles['R Hip'] = self._calculate_joint_angle(self.X[torso_idx], self.X[rthigh_idx],
                                                                 self.IMU2anatomical_rot['Torso'], self.IMU2anatomical_rot['R Thigh'])
        
        # Knee angle estimates, saggital flexion/extension is stored in first dimension
        self.joint_angles['L Knee'] = self._calculate_joint_angle(self.X[lthigh_idx], self.X[lshank_idx],
                                                                 self.IMU2anatomical_rot['L Thigh'], self.IMU2anatomical_rot['L Shank'])
        self.joint_angles['R Knee'] = self._calculate_joint_angle(self.X[rthigh_idx], self.X[rshank_idx],
                                                                 self.IMU2anatomical_rot['R Thigh'], self.IMU2anatomical_rot['R Shank'])
        
        # Knee angle estimates, dorsiflexion/plantarflexion is stored in first dimension
        self.joint_angles['L Ankle'] = self._calculate_joint_angle(self.X[lshank_idx], self.X[lfoot_idx],
                                                                 self.IMU2anatomical_rot['L Shank'], self.IMU2anatomical_rot['L Foot'])
        self.joint_angles['R Ankle'] = self._calculate_joint_angle(self.X[rshank_idx], self.X[rfoot_idx],
                                                                 self.IMU2anatomical_rot['R Shank'], self.IMU2anatomical_rot['R Foot'])


