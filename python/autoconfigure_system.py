import apdm

context = apdm.apdm_ctx_allocate_new_context()

try:
    apdm.apdm_ctx_open_all_access_points(context)

    """ streaming configuration """
    streaming_config = apdm.apdm_streaming_config_t()
    apdm.apdm_init_streaming_config(streaming_config)
    streaming_config.enable_accel = True
    streaming_config.enable_gyro = True
    streaming_config.enable_mag = True
    streaming_config.wireless_channel_number = 80
    r = apdm.apdm_ctx_autoconfigure_devices_and_accesspoint_streaming(context, streaming_config)
    if r != apdm.APDM_OK:
        raise Exception("Unable to autoconfigure system, r = " + apdm.apdm_strerror(r));

finally:
    apdm.apdm_ctx_disconnect(context) 
    apdm.apdm_ctx_free_context(context)

