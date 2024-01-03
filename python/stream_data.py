import apdm
 
try:
    num_aps = apdm.uintArray(1)
    return_code = apdm.apdm_ap_get_num_access_points_on_host1(num_aps)
    # print("return code is " + str(return_code))
    # print("Number of APs on host is " + str(num_aps))

    context = apdm.apdm_ctx_allocate_new_context()
    # print(context)
    if context == None:
        raise Exception("unable to allocate new context")
    
    r = apdm.apdm_ctx_open_all_access_points(context)
    if r != apdm.APDM_OK:
        raise Exception("unable to open all access points")
    
    r = apdm.apdm_ctx_sync_record_list_head(context)
    if r != apdm.APDM_OK:
        raise Exception("unable to sync record head list")
    
    #DEVICE ID LIST
    numSensors = apdm.uintArray(1)
    retCode = apdm.apdm_ctx_get_expected_number_of_sensors2(context, numSensors)
    device_ids = []
    for x in range(numSensors[0]):
        device_ids.append(apdm.apdm_ctx_get_device_id_by_index(context, x))

    #Attempt to read data 10000 times. This may not equal the number of samples, however
    #as there may not be data available to read each time if reading faster than
    #the sampling rate
    for i in range(10000):
        r = apdm.apdm_ctx_get_next_access_point_record_list(context);
        if r != apdm.APDM_OK:
            if r != apdm.APDM_NO_MORE_DATA:
                raise Exception("error while getting next access point record list")
            
            apdm.apdm_usleep(50000)
            continue
        
        raw_record = apdm.apdm_record_t()
        for device_id in device_ids:
            ret = apdm.apdm_ctx_extract_data_by_device_id(context, device_id, raw_record);
            print("Received data status code: " + str(ret))
            print("Rec.sync_val64: " + str(raw_record.sync_val64))
            print("Rec.accl_x_axis_si: " + str(raw_record.accl_x_axis_si))
            print("Rec.accl_y_axis_si: " + str(raw_record.accl_y_axis_si))
            print("Rec.accl_z_axis_si: " + str(raw_record.accl_z_axis_si))
            print("Rec.gyro_x_axis_si: " + str(raw_record.gyro_x_axis_si))
            print("Rec.gyro_y_axis_si: " + str(raw_record.gyro_y_axis_si))
            print("Rec.gyro_z_axis_si: " + str(raw_record.gyro_z_axis_si))
        
finally:
    # Disconnect handles from AP's
    apdm.apdm_ctx_disconnect(context) 

    # Free memory used by context
    apdm.apdm_ctx_free_context(context)
    context = None


