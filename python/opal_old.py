import apdm
from time import time 
from datetime import datetime
import csv
import sys, os
from contextlib import contextmanager
import contextlib

class Opal:
    def __init__(self):
        self.context = None
        self.n_aps = 0              # number of access points
        self.n_sensors = 0          # number of sensors 
        self.device_ids = []
        self.streaming_config = None
        self.sensor_data_csv = None

    def autoconfigure(self):
        """
        Autoconfigures access points and streaming configurations. 
        Adapted from APDM SDK autoconfigure_system.py.
        """
        self.context = apdm.apdm_ctx_allocate_new_context()

        try:
            apdm.apdm_ctx_open_all_access_points(self.context)

            """ streaming configuration """
            self.streaming_config = apdm.apdm_streaming_config_t()
            apdm.apdm_init_streaming_config(self.streaming_config)
            self.streaming_config.enable_accel = True
            self.streaming_config.enable_gyro = True
            self.streaming_config.enable_mag = True
            self.streaming_config.wireless_channel_number = 80

            r = apdm.apdm_ctx_autoconfigure_devices_and_accesspoint_streaming(self.context, self.streaming_config)
            if r != apdm.APDM_OK:
                raise Exception("Unable to autoconfigure system, r = " + apdm.apdm_strerror(r))
            
        finally:
            apdm.apdm_ctx_disconnect(self.context) 
            apdm.apdm_ctx_free_context(self.context)
        
        
    def start_streaming(self):
        """
        Start streaming for Opal sensors in preparation to call get_next() in a loop to continuously get sensor data. 
        Adapted from APDM SDK stream_data.py.
        """

        self.n_aps = apdm.uintArray(1)
        return_code = apdm.apdm_ap_get_num_access_points_on_host1(self.n_aps)

        # Create sensor "context"
        self.context = apdm.apdm_ctx_allocate_new_context()
        if self.context == None:
            raise Exception("Unable to allocate new context")
        
        # Open access points
        r = apdm.apdm_ctx_open_all_access_points(self.context)         
        if r != apdm.APDM_OK:
            raise Exception("Unable to open all access points")
        
        # Sync sensors and access points together
        r = apdm.apdm_ctx_sync_record_list_head(self.context)
        if r != apdm.APDM_OK:
            raise Exception("Unable to sync record head list")
        
        # Get device IDs for all sensors currently active 
        self.n_sensors = apdm.uintArray(1)
        retCode = apdm.apdm_ctx_get_expected_number_of_sensors2(self.context, self.n_sensors)
        
        for x in range(self.n_sensors[0]):
            self.device_ids.append(apdm.apdm_ctx_get_device_id_by_index(self.n_sensors, x))

    ###########################################################################################
    # The following functions were adapted from https://github.com/aero-man/apdm-opal-simple-stream/tree/master, a publically available streaming tool for APDM Opal sensors. 
    # Functions were mainly adapted from sensor_stream.py
    
    def get_next(self):
        """
        Get the next record for each sensor.
        Returns:
            A 2D list with one row for each active sensor. Each row has the following data:
                - Computer time in milliseconds
                - Opal sensor Unix time in in milliseconds 
                - Device/sensor serial number 
                - Accelerometer X, Y, and Z
                - Gyroscope X, Y, and Z
                - Magnometer X, Y, and Z
        NOTE: A record is not always returned as there may not be data available to read each time if reading is faster than sampling rate.
        """
        raw_record = self._get_next_record_from_sensor()
        clean_record = self._raw_record_to_gyro_accel_and_mag(raw_record)
        return clean_record

    def _get_next_record_from_sensor(self):
        """
        Helper function to get next raw record from sensor
        """
        return_code = apdm.apdm_ctx_get_next_access_point_record_list(self.context)
        if return_code != apdm.APDM_OK:
            apdm.apdm_usleep(16666) # microseconds
        raw_record = apdm.apdm_record_t()
        return raw_record

    def _raw_record_to_gyro_accel_and_mag(self, raw_record):
        """
        Helper function to format raw record into list of:
            - Unix time in milliseconds
            - Device/sensor serial number
            - Accelerometer X, Y, and Z
            - Gyroscope X, Y, and Z
            - Magnetometer X, Y, and Z
        """
        sensor_data = []
        for device_id in self.device_ids:
            return_code = apdm.apdm_ctx_extract_data_by_device_id(self.context, device_id, raw_record)
            single_device_data = [
                        str(time()), # Computer's Unix time in milliseconds
                        str(raw_record.v2_sync_val64_us), # APDM sensors' Unix time in milliseconds
                        str(raw_record.device_info_serial_number),
                        str(raw_record.accl_x_axis_si),
                        str(raw_record.accl_y_axis_si),
                        str(raw_record.accl_z_axis_si),
                        str(raw_record.gyro_x_axis_si),
                        str(raw_record.gyro_y_axis_si),
                        str(raw_record.gyro_z_axis_si),
                        str(raw_record.mag_x_axis_si),
                        str(raw_record.mag_y_axis_si),
                        str(raw_record.mag_z_axis_si)]
            sensor_data.append(single_device_data)
        return sensor_data

    ###########################################################################################

    def init_csv(self):
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        csv_name = 'trial_{0}.csv'.format(timestamp)
        file = open(csv_name, 'w')
    
        column_names = [['computer_unix_time_ms','sensor_unix_time_ms', 'device_id', 
                        'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z',
                        'mag_x', 'mag_y', 'mag_z']]
        
        self.sensor_data_csv = csv.writer(file)
        self.sensor_data_csv.writerow(column_names)
    
    def write(self, sensor_data):
        for device_data in sensor_data:
            self.sensor_data_csv.writerow(device_data)

    def stop_streaming(self):
        apdm.apdm_ctx_disconnect(self.context) 
        apdm.apdm_ctx_free_context(self.context)

def main(user_option):
    opal = Opal()

    if user_option == "configure":
            opal.autoconfigure()

    elif user_option == "stream":
        opal.init_csv()
        opal.start_streaming()

        while True:
            try:
                sensor_data = opal.get_next()
                opal.write(sensor_data)
            except KeyboardInterrupt:
                print("Shutdown request. Exiting...")
                opal.stop_streaming()

    else:
        print("Invalid option, choose configure or stream.")

if __name__ == '__main__':
    user_option = sys.argv[1]
    if user_option == "configure" or user_option == "stream":
        main(user_option)