import sys
import csv
from simple_stream import app_logger as AppLogger
from simple_stream import sensor_config as SensorConfig
from simple_stream import sensor_stream as SensorStream
from simple_stream import stream_csv_writer as StreamCsvWriter
from sklearn.decomposition import PCA
import numpy as np

class Opal:
    def __init__(self):
        self.stream = SensorStream()
        self.csv_writer = StreamCsvWriter()
        self.logger = AppLogger()
        self.window_size = 100
        self.update_rate = 100 # Hz
        self.dt = 1 / self.update_rate # seconds
        self.data = []
        self.n_sensors = None
        self.device_ids = None

    def start_streaming(self):
        self.stream.start()
        self.n_sensors = len(self.stream.device_ids)
        self.device_ids = self.stream.device_ids
        for device_id in self.device_ids:
            self.data.append(np.empty((0, 6)))                          # initialize list of empty arrays for data, 6 columns (x,y,z gyro and accel)

    def update_data(self):
        try:
            sensor_data = self.stream.get_next()                        # outputs data in order of device_ids
            self.logger.logger.debug("Received sensor data: {0}".format(sensor_data))
            self.csv_writer.write(sensor_data)
            self._append_data(sensor_data)
        except Exception as e:
            self.logger.logger.error("Could not retrieve sensor data. Error: {0}"
                .format(e))
    
    def _append_data(self, sensor_data):
        for idx, device_data in enumerate(sensor_data):
            self.data[idx] = np.append(self.data[idx], device_data[2:7])   # only extract and store gyro and accel data 
            # need to limit array size to window_size

    def EKF(self):
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

        return

    def calibrate_axis(self):
        pass