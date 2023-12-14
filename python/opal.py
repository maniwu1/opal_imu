import apdm
import numpy as np 

class Opal:
    def __init__(self):
        self.opal = None
        self.n_aps = 0              # number of access points
        self.n_sensors = 0          # number of sensors 
        self.device_ids = []
        self.streaming_config = None

    def autoconfigure(self):
        apdm.apdm_ctx_open_all_access_points(self.opal)

        """ streaming configuration """
        self.streaming_config = apdm.apdm_streaming_config_t()
        apdm.apdm_init_streaming_config(self.streaming_config)
        self.streaming_config.enable_accel = True
        self.streaming_config.enable_gyro = True
        self.streaming_config.enable_mag = True
        self.streaming_config.wireless_channel_number = 80

        r = apdm.apdm_ctx_autoconfigure_devices_and_accesspoint_streaming(self.opal, self.streaming_config)
        if r != apdm.APDM_OK:
            raise Exception("Unable to autoconfigure system, r = " + apdm.apdm_strerror(r))

    def start_streaming(self):
        self.n_aps = apdm.uintArray(1)
        return_code = apdm.apdm_ap_get_num_access_points_on_host1(self.n_aps)
        print("Return code:", return_code)
        print("Number of APs on host:", self.n_aps)

        self.opal = apdm.apdm_ctx_allocate_new_context()
        print(self.opal)
        if self.opal == None:
            raise Exception("Unable to allocate new context")
        
        r = apdm.apdm_ctx_open_all_access_points(self.opal)         # should I store the status?
        if r != apdm.APDM_OK:
            raise Exception("Unable to open all access points")
        
        r = apdm.apdm_ctx_sync_record_list_head(self.opal)
        if r != apdm.APDM_OK:
            raise Exception("Unable to sync record head list")
        
        #DEVICE ID LIST
        self.n_sensors = apdm.uintArray(1)
        retCode = apdm.apdm_ctx_get_expected_number_of_sensors2(self.opal, self.n_sensors)
        
        for x in range(self.n_sensors[0]):
            self.device_ids.append(apdm.apdm_ctx_get_device_id_by_index(self.n_sensors, x))

    def log_data(self):
        pass

    def stop_streaming(self):
        apdm.apdm_ctx_disconnect(self.opal) 
        apdm.apdm_ctx_free_context(self.opal)