#ifndef APDM_H_
#define APDM_H_

/**
 * @file apdm_docs
 */

/**
 * @defgroup Setup
 * @defgroup Context
 * @defgroup AccessPoint
 * @defgroup DataFiles
 * @defgroup Monitor
 * @defgroup MonitorCommands
 * @defgroup DockingStation
 * @defgroup DataHandling
 * @defgroup Logging
 * @defgroup Misc
 */

#ifdef __cplusplus
extern "C" {
#endif


#include "apdm_stdincludes.h"
#include "apdm_types.h"

#ifdef IGNORE_DEPRECATED
#  define APDM_DEPRECATED
#else
#  ifdef __GNUC__
#    define APDM_DEPRECATED   __attribute__ ((deprecated))
#  else
#    define APDM_DEPRECATED
#  endif
#endif

/**
 * @param *dest Destination into which to store the number of USB access points attached to the host based on the VID/PID listing from the OS.
 *
 * @return APDM_OK on success, error code otherwise
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_num_access_points_on_host1(uint32_t *dest);
APDM_EXPORT int apdm_v1_ap_get_num_accesspoints_on_host(uint32_t *dest);
APDM_EXPORT int apdm_v2_get_num_dockingstations_on_host(uint32_t *num_devices);
APDM_EXPORT int apdm_v2_ap_get_num_accesspoints_on_host(uint32_t *dest);

/**
 * Used to connect to an access point.
 *
 * @param ap_handle An un-configured access point handle.
 * @param indexNumber The index number, starting at zero, of the AP on the host to which to connect.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_connect(apdm_ap_handle_t ap_handle, const int indexNumber);

/**
 * Disconnects the access point handle from the underlying OS binding
 *
 * @param ap_handle The handle to be disconnected.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_disconnect(apdm_ap_handle_t ap_handle);

/**
 * Initializes an access point handle
 *
 * @param ap_handle Pointer to the handle to be initialized
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_init_handle(apdm_ap_handle_t ap_handle);


//Device L0
/**
 * Closes and de-allocates a device handle.
 *
 * @param device_handle The handle to be closed and deallocated
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_close_and_free(apdm_device_handle_t device_handle);

/**
 * Allocates and opens/connects to a sensor on the system (monitor should be plugged into the docking station).
 *
 * @param sensor_index The index of the sensor attached to the host. This is the position of the dock in the chain, starting from zero.
 *
 * @return Zero handle on error, non-zero handle on success.
 * @ingroup Monitor
 */
APDM_EXPORT apdm_device_handle_t apdm_sensor_allocate_and_open(const uint32_t sensor_index);

/**
 * Frees memory associated with a device handle
 *
 * @param device_handle The handle to be freed.
 * @ingroup Monitor
 */
APDM_EXPORT void apdm_sensor_free_handle(apdm_device_handle_t device_handle);

/**
 * Allocates memory and returns a new sensor handle.
 *
 * @return Zero on failure, non-zero handle on success.
 * @ingroup Monitor
 */
APDM_EXPORT apdm_device_handle_t apdm_sensor_allocate_handle(void);

/**
 * Opens a sensor by it's corresponding index number on the host computer.
 *
 * @param device_handle The handle to which the corresponding sensor is to be associated with
 * @param device_index The index of the device which is to be opened
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_open(apdm_device_handle_t device_handle, const uint32_t device_index);

/**
 * Fills the buffer pointed to by serial_number_buffer with a list of Motion Monitor ID numbers.
 *
 * @param *serial_number_buffer Destination array into which device IDs are to be populated
 * @param buffer_length The maximum number of entries in the serial_number_buffer buffer.
 * @param *dest_count The number of devices added to the buffer list
 *
 * @return APDM_OK on success, MONITOR_READ_TIMEOUT_ERROR if one (or more) of the docks does not have 
 *          a monitor plugged in, otherwise error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_list_attached_sensors3(uint32_t *serial_number_buffer, const uint32_t buffer_length, uint32_t *dest_count);


/**
 * @param *dest_num_docks Destination into which to store the number of docking stations attached to the host
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_sensor_get_num_attached_dockingstations1(uint32_t *dest_num_docks);
APDM_EXPORT int apdm_ds_sensor_get_num_attached_dockingstations2(uint32_t *dest_num_docks);
APDM_EXPORT int apdm_v2_get_num_monitors_on_host(uint32_t *dest); 


/**
 * Puts the opal into a mode where it will generate various classes of erroronious output data for QA testing purposes.
 *
 * @param h The device handle
 * @param enable_flag Turn on or off artificial generation of errornious data.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 */
APDM_EXPORT int apdm_v2_data_error_mode(apdm_device_handle_t h, const bool enable_flag);

/**
 * Clears all the error states and counts on a V2 opal.
 *
 * @param h The device handle
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 */
APDM_EXPORT int apdm_v2_clear_all_errors(apdm_device_handle_t h);

/**
 * Closes the handle.
 *
 * @param device_handle The handle to be closed
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_close(apdm_device_handle_t device_handle);

/**
 * this function is used to verify that the given device handle has a version of calibration data
 * that is supported by the libraries.
 *
 * @param device_handle The device handle
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_verify_supported_calibration_version(apdm_device_handle_t device_handle);


/**
 * Only relevant to windows. Sets the directory name into which correlation fifo temp files should be located.
 *
 * @param *directory Directory into which fifo files should be placed, with trailing slash.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_set_correlation_fifo_temp_directory(const char *directory);

/**
 * Allows you to override the minimum calibration version number used to validate calibration versions on motion sensors.
 *
 * @param new_version Version number, e.g. 4 Set this to zero to use library default version number.
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Setup
 */
APDM_EXPORT int apdm_calibration_override_minimum_supported_version(const uint32_t new_version);

/**
 * this function is used to verify that the given device handle has a version of firmware
 * that is supported by the libraries.
 *
 * @param device_handle The device handle
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_verify_supported_version(apdm_device_handle_t device_handle);

/**
 * Allows you to override the minimum motion sensor version number used to validate motion sensor versions.
 *
 * @param new_version Version number, e.g. "2010-09-02" Set this to NULL to use library default version number.
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_override_minimum_supported_version(const char *new_version);

/**
 * this function is used to verify that the given docking station handle has a version of firmware
 * that is supported by the libraries.
 *
 * @param device_handle The device handle
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_comm_channel_verify_supported_version(apdm_device_handle_t device_handle);

/**
 * Allows you to override the minimum docking station version number used to validate dock versions.
 *
 * @param new_version Version number, e.g. 20100902170629 Set this to zero to use library default version number.
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_override_minimum_supported_version(const uint64_t new_version);

//AP L1


/**
 * Retrieves the latency of an individual monitor from the given AP.
 *
 * @param ap_handle The ap handle which is to be queried.
 * @param monitor_id The ID of the monitor for which you want to know the latency.
 * @param *dest The destination into which to store the latency value.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_monitor_latency(apdm_ap_handle_t ap_handle, const uint32_t monitor_id, int64_t *dest);


/**
 * @param ap_handle The access point handle
 * @param delta_threshold The threshold, in milliseconds, for the AP to start blinking green/blue if one (or more)
 *                        monitors are falling behind in their transmission (e.g. out of range).
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_set_warning_blink_threshold(apdm_ap_handle_t ap_handle, const uint32_t delta_threshold);

/**
 * @param ap_handle The access point handle
 * @param delta_threshold The threshold, in milliseconds, for the AP to start blinking green/red if one (or more)
 *                        monitors are falling behind in their transmission (e.g. out of range).
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_set_error_blink_threshold(apdm_ap_handle_t ap_handle, const uint32_t delta_threshold);

/**
 * @param ap_handle The AP handle
 * @param *dest The destination into which to store the LED status, of type apdm_ap_wireless_streaming_status_t
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_wireless_streaming_led_status(apdm_ap_handle_t ap_handle, uint32_t *dest);

/**
 * @param streaming_status Streaming status, of type apdm_ap_wireless_streaming_status_t, for which you want the string representation.
 *
 * @return Pointer to a string for the given status type
 * @ingroup AccessPoint
 */
APDM_EXPORT const char* apdm_ap_wireless_streaming_status_t_str(const apdm_ap_wireless_streaming_status_t streaming_status);

/**
 * Sets the maximum latency of packets that should be coming from devices to the access point.
 *
 * @param ap_handle The AP handle for which this value is to be set.
 * @param max_latency_seconds The maximum delay, in seconds, which a device should send buffered packets to the AP. A value of APDM_INFINITE_MAX_LATENCY implies infinity.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int adpm_ap_set_max_latency_value_seconds(apdm_ap_handle_t ap_handle, const uint16_t max_latency_seconds);

/**
 * @param ap_handle The AP handle for which this value is to be set.
 * @param minimum_sync_value The minimum sync value that you want sensors to send out. This is useful for skipping ahead in a data stream.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int adpm_ap_set_minimum_sync_value(apdm_ap_handle_t ap_handle, const uint64_t minimum_sync_value);

/**
 * @param ap_handle The AP handle for which this value is to be set.
 * @param *minimum_sync_value This will be populated with the miniumum sync values that sensors are supposed to be sending out.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int adpm_ap_get_minimum_sync_value(apdm_ap_handle_t ap_handle, uint64_t *minimum_sync_value);


/**
 * @param ap_handle The AP handle.
 * @param gpio_pin The pin in question (see the apdm_ap_gpio_pin_t enum in apdm_types.h). Use APDM_AP_GPIO_0 to control the digital
 *                 input or output pins on the DIN-6 connector. Use APDM_AP_ANALOG_OUT_0 or APDM_AP_ANALOG_IN_0 to control or read the
 *                 analog input/output pins on the DIN-4 connector.
 * @param *output_value Destination into which to store the current value of the GPIO pin.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_io_value(apdm_ap_handle_t ap_handle, const apdm_ap_gpio_pin_t gpio_pin, uint32_t *output_value);



/**
 * @param ap_handle The AP handle.
 * @param gpio_pin The pin in question (see the apdm_ap_gpio_pin_t enum in apdm_types.h). Use APDM_AP_GPIO_0 to control the digital
 *                 input or output pins on the DIN-6 connector. Use APDM_AP_ANALOG_OUT_0 or APDM_AP_ANALOG_IN_0 to control or read the
 *                 analog input/output pins on the DIN-4 connector.
 * @param output_value New value to set on a GPIO pin that has been configured as an output pin. When setting an
 *                     analog output value, this will be a number between 0 and 1023, that gets set on the DAC
 *                     and puts out between 0 and 5 volts.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_set_io_value(apdm_ap_handle_t ap_handle, const int gpio_pin, const uint32_t output_value);

/**
 * @param context The context of communications.
 * @param ap_id The ID number of the AP to manipulate the GPIO on.
 * @param gpio_pin The pin in question (see the apdm_ap_gpio_pin_t enum in apdm_types.h). Use APDM_AP_GPIO_0 to control the digital
 *                 input or output pins on the DIN-6 connector. Use APDM_AP_ANALOG_OUT_0 or APDM_AP_ANALOG_IN_0 to control or read the
 *                 analog input/output pins on the DIN-4 connector.
 * @param *output_value Destination into which to store the current value of the GPIO pin.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_ap_get_io_value(apdm_ctx_t context, const uint32_t ap_id, const apdm_ap_gpio_pin_t gpio_pin, uint32_t *output_value);

/**
 * Allows for querying of fields from a sync box attached to a V2 access point
 *
 * @param context The context of communications.
 * @param ap_id The ID number of the AP to query.
 * @param @cmd The value type to be queried for.
 * @param *output_value Destination into which to store the query response.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_ap_sync_box_query(apdm_ctx_t context, const uint32_t ap_id, const can_query_cmd_t cmd, uint32_t *output_value);

/**
 * @param context The context of communications.
 * @param ap_id The ID number of the AP to manipulate the GPIO on.
 * @param gpio_pin The pin in question (see the apdm_ap_gpio_pin_t enum in apdm_types.h). Use APDM_AP_GPIO_0 to control the digital
 *                 input or output pins on the DIN-6 connector. Use APDM_AP_ANALOG_OUT_0 or APDM_AP_ANALOG_IN_0 to control or read the
 *                 analog input/output pins on the DIN-4 connector.
 * @param output_value New value to set on a GPIO pin that has been configured as an output pin.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_ap_set_io_value(apdm_ctx_t context, const uint32_t ap_id, const apdm_ap_gpio_pin_t gpio_pin, const uint32_t output_value);

/**
 * @param context The context of communications.
 * @param ap_id The ID number of the AP to manipulate the GPIO on.
 *
 * @param *output_value Destination into which to store the status, 1 if a sync box is attached, 0 if not. This only works with V2 hardware. 0 will be returned for V1 hardware.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_ap_sync_box_attached(apdm_ctx_t context, const uint32_t ap_id, uint32_t *dest);

/**
 * @param context The context of communications.
 * @param minimum_sync_value The minimum sync value that you want sensors to send out. This is useful for skipping ahead in a data stream.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_set_minimum_sync_value(apdm_ctx_t context, const uint64_t minimum_sync_value);




/**
 * Returns the number of sensors that are configured in the context. This is with respect to an already-configured context.
 * It is not necessarily the number of sensors attached to the system.
 *
 * @param context The context of communications.
 *
 * @return The number of sensors configured in the context.
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_expected_number_of_sensors2(apdm_ctx_t context, uint32_t *dest);







/**
 * Sends a string-command to the access point, mostly used for debugging and non-standard functionality.
 *
 * @param ap_handle The handle for the AP to which the command is to be sent
 * @param *cmdToSend The command to send
 * @param *BYTE_ARRAY The destination into which the response from the AP is to be placed.
 * @param outputBufferLength The length of the outputStringBuffer.
 * @param numLinesToRead The number of lines that are expected in response to the command being sent.
 * @param timeoutMilliseconds Maximum time length to wait for the response.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_send_accesspoint_cmd(apdm_ap_handle_t ap_handle,
		const char *cmdToSend, char *BYTE_ARRAY, const uint32_t outputBufferLength,
		const uint32_t numLinesToRead, const uint32_t timeoutMilliseconds);


//AP L2
/**
 * Returns the firmware version string from the access point.
 *
 * @param ap_handle The handle with respect to what version string you want.
 * @param *BYTE_ARRAY The destination buffer into which you want the version string copied into, must not be NULL.
 * @param destLength The maximum length of the destination string, must be greater then zero.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_version_string(apdm_ap_handle_t ap_handle, char *BYTE_ARRAY, const int destLength);


/**
 *
 * @param ap_handle The access point handle for which you want the numeric representation of the version
 * @param *dest The destination into which to store the ap firmware version number
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_version(apdm_ap_handle_t ap_handle, uint64_t *dest);




/**
 * Returns the board hardware version string from the access point.
 *
 * @param ap_handle The handle with respect to what version string you want.
 * @param *BYTE_ARRAY The destination buffer into which you want the version string copied into, must not be NULL.
 * @param destLength The maximum length of the destination string, must be greater than 0.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_board_version_string(apdm_ap_handle_t ap_handle, char *BYTE_ARRAY, const int destLength);

/**
 *
 * @param *ap_handle The access point handle
 * @param *dest_id The destination into which to store the ID of the access point
 * @param *dest_board_version The destination into which to store the printed circuit board version
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup AccessPoint
 */
APDM_EXPORT  int apdm_ap_get_id_and_board_version(apdm_ap_handle_t ap_handle, uint32_t *dest_id, uint32_t *dest_board_version);

/**
 * this function is used to verify that the given access point has a version of firmware
 * that is supported by the libraries.
 *
 * @param ap_handle The access point handle
 *
 * @return APDM_OK if the version is OK, respective error code otherwise.
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_verify_supported_version(apdm_ap_handle_t ap_handle);

/**
 * Allows you to override the minimum access point station version number used to validate AP versions.
 *
 * @param new_version Version number, e.g. 20100902170629 Set this to zero to use library default version number.
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_override_minimum_supported_version(const uint64_t new_version);


/**
 * This returns the serial number of the access point
 *
 * @param ap_handle The AP handle associated with the AP for which you want the serial number.
 * @param *dest The destination into which the serial number is to be stored.
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_id(apdm_ap_handle_t ap_handle, uint32_t *dest);

/**
 * Retrieves the case ID of the AP.
 *
 * @param ap_handle The AP handle
 * @param *BYTE_ARRAY The destination buffer into which to store the case ID string.
 * @param dest_buffer_length The max length of the destination buffer
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_case_id(apdm_ap_handle_t ap_handle, char *BYTE_ARRAY, const int dest_buffer_length);

//FIXME document this
APDM_EXPORT int apdm_ap_get_new_firmware_version_ok_flag(apdm_ap_handle_t ap_handle, uint32_t *dest);

//FIXME document this
APDM_EXPORT int apdm_ap_check_new_firmware_version(apdm_ap_handle_t ap_handle, const int64_t new_firmware_version);

/**
 * This function will reset the given access point into bootloader
 *
 *
 * @param *ap_handle A handle that is already connected to an AP via usb.
 *
 * @return APDM_OK on success, and if successful, the handle will have been DISCONNECTED and you
 *         must re-connect to the AP.
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_reset_into_bootloader(apdm_ap_handle_t ap_handle);

/**
 * This function will reset the given access point into firmware
 *
 *
 * @param *ap_handle A handle that is already connected to an AP via usb.
 *
 * @return APDM_OK on success, and if successful, the handle will have been DISCONNECTED and you
 *         must re-connect to the AP.
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_reset_into_firmware(apdm_ap_handle_t ap_handle);

/**
 * Frees memory for the given access point handle
 *
 * @param ap_handle The handle to be freed
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_free_handle(apdm_ap_handle_t ap_handle);

/**
 * Allocates memory for an access point handle.
 *
 * @return NULL on failure, non-NULL on success.
 * @ingroup AccessPoint
 */
APDM_EXPORT apdm_ap_handle_t apdm_ap_allocate_handle(void);


/**
 *
 * @param *ap_handle A handle that is already connected to an AP via usb.
 *
 * @return APM_FIRMWARE if the AP is in firmware mode,
 *         APM_BOOTLOADER is it's in bootloader,
 *         APM_UNKNOWN if the mode cannot be determined,
 *
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_mode(apdm_ap_handle_t ap_handle);


/**
 * @param ap_handle The AP handle
 * @param *dest_protocol_subversion The destination into which to store the protocol version
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ap_get_protocol_subversion(apdm_ap_handle_t ap_handle, int64_t *dest_protocol_subversion);

/**
 * Will cause all access points connected to the host to be opened and associated with the passed handle.
 * Note: Accesspoints can only be opened by one application at a time. If there are other applications, such as Motion Studio running
 * that have already open the attached Accesspoints, then this will fail to open them.
 *
 * @param context The handle for which to associate all opened access points.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_open_all_access_points(apdm_ctx_t context);

/**
 * Sets the error handling behavior of the underlying APDM libraries. Particularly affects when errors, partial records or full records are returned from
 * a call to getting a record list.
 *
 * @param context The apdm context
 * @param new_mode An enum APDMErrorHandlingBehavior indicating what error handling mode to set.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT enum APDM_Status apdm_ctx_set_error_handling_mode(apdm_ctx_t context, enum APDMErrorHandlingBehavior new_mode);


/**
 * @param context The apdm context
 * @param *dest_comp_data The destination into which to store compensation data for the sensor of index sensor_index.
 * @param sensor_index The index of the sensor for which you want to retrieve compensation data.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_sensor_compensation_data(apdm_ctx_t context, apdm_sensor_compensation_t *dest_comp_data, const int32_t sensor_index);

/**
 * @param context The apdm context
 * @param *src_comp_data The source of compensation data which to store into the context for the sensor of index sensor_index.
 * @param sensor_index The index of the sensor for which you want to retrieve compensation data.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_set_sensor_compensation_data(apdm_ctx_t context, const apdm_sensor_compensation_t *src_comp_data, const int32_t sensor_index);

/**
 * Depending on the output rate (e.g. 128 samples per second, 80 samples per second etc), this will return the expected
 * sync delta between any two samples.
 *
 * @param context The apdm context
 * @param *dest_expected_sync_delta The destination into which to store the expected sync delta between any two samples
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_expected_sync_delta(apdm_ctx_t context, uint16_t *dest_expected_sync_delta);


/**
 * Metadata can be stored in the context with respect to a given device id, and later retrieved.
 *
 * @param context The apdm handle
 * @param device_id The device ID to set the meta data for (this is different then the Case ID on the back of the monitor).
 * @param value The uint32_t type meta data to be stored. You can optionally associate meta-data with a device ID in a context.
 *               It's an arbitrary number for which the meaning is defined by the application using the library.
 *               E.G. You could use this to specify what limb of a persons body each motion monitor is attached to.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_set_metadeta_uint32(apdm_ctx_t context, const uint32_t device_id, const uint32_t value );

/**
 * Metadata can be stored in the context with respect to a given device id, and later retrieved.
 *
 * @param context The apdm handle
 * @param device_id The device ID for which you want the metadata string (this is different then the Case ID on the back of the monitor).
 * @param str The char* type meta data to be stored, maximum length is USER_META_DATA_STRING_SIZE(64)
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_set_metadata_string(apdm_ctx_t context, const uint32_t device_id, const char *str);

APDM_EXPORT char* apdm_ctx_get_metadata_string(apdm_ctx_t context, const uint32_t device_id);

/**
 * Allows for the retrieval of metadata for a device id.
 *
 * @param context The apdm handle
 * @param device_id The device_id for which you want metadata (this is different then the Case ID on the back of the monitor)
 *
 * @return The data associated with the device id, or zero if an error or no data having been set.
 * @ingroup Context
 */
APDM_EXPORT uint32_t apdm_ctx_get_metadata_uint32(apdm_ctx_t context, const uint32_t device_id);


/**
 * @param context The apdm handle
 * @param *dest The destination into which to store the configured wireless mode. The value will be from the enum apdm_wireless_mode_t
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_wireless_configuration_mode(apdm_ctx_t context, int *dest);

/**
 * Gets device detailed information about the device id passed in.
 *
 * @param context The apdm handle
 * @param device_id The ID of the device for which you'd like to get data (this is different then the Case ID on the back of the monitor)
 * @param *dest The destination structure into which you'd like to store the data
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_device_info(apdm_ctx_t context, const uint32_t device_id, apdm_device_info_t *dest);

/**
 * @param context
 *
 * @return The number of access points attached to the host
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_num_access_points_found(apdm_ctx_t context);

/**
 * @param context
 * @param ap_index The AP index number for which you want AP ID. This should be greater then or equal to 0 and less than the number of APs configured.
 * @param *dest Destionation address into which the AP ID should be stored.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_ap_id_for_ap_index(apdm_ctx_t context, const int ap_index, uint32_t *dest);

/**
 * @return The total number of sample lists collected since the handle was initialized.
 * @ingroup Context
 */
APDM_EXPORT uint32_t apdm_ctx_get_num_sample_lists_collected(apdm_ctx_t context);

/**
 * @return The total number of samples collected since the handle was initialized.
 * @ingroup Context
 */
APDM_EXPORT uint32_t apdm_ctx_get_num_samples_collected(apdm_ctx_t context);

/**
 * @param device_id Device ID to get the number of samples for (this is different then the Case ID on the back of the monitor)
 * @return The total number of samples collected since the handle was initialized for the device id specified
 * @ingroup Context
 */
APDM_EXPORT uint32_t apdm_ctx_get_num_samples_collected_from_device(apdm_ctx_t context, const uint32_t device_id);

/**
 * @return The number of omitted sample sets since the last time the context was initialized or since the last time apdm_sync_record_head_list() was called.
 * @ingroup Context
 */
APDM_EXPORT uint32_t apdm_ctx_get_total_omitted_sample_sets(apdm_ctx_t context);

/**
 * @return The number of omitted sample sets since the most recently requested sample set and the previously retrieved sample set.
 * @ingroup Context
 */
APDM_EXPORT uint32_t apdm_ctx_get_num_omitted_sample_sets(apdm_ctx_t context);

/**
 * @return The number of omitted samples between the most recently requested sample set and the previously retrieved sample set.
 * @ingroup Context
 */
APDM_EXPORT uint32_t apdm_ctx_get_num_omitted_samples(apdm_ctx_t context);

/**
 * @return The number of omitted samples since the context was initialized, or since the last time apdm_sync_record_head_list() was called.
 * @ingroup Context
 */
APDM_EXPORT uint32_t apdm_ctx_get_total_omitted_samples(apdm_ctx_t context);

/**
 * @param context
 * @param *dest Destination into which to store the sampling frequency
 *
 * @return Returns the sampling frequency that the devices are running at
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_sampling_frequency(apdm_ctx_t context, uint32_t *dest);

/**
 * Gets data for a particular device id from the most record list.
 *
 * @param context
 * @param device_id  The device id for which you want to retrieve data for (this is different then the Case ID on the back of the monitor)
 * @param *dest The destination into which to put data.
 *
 * @return APDM_OK on success,  APDM_NO_MORE_DATA if no more data, error code otherwise.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_extract_data_by_device_id(apdm_ctx_t context, const uint32_t device_id, apdm_record_t *dest);

/**
 * Gets the next record from the access point indicated
 *
 * @param context
 * @param *data Destination into which to place data. This can be null, if you want to only trigger transfers from the
 *              AP, but will always return APDM_NO_MORE_DATA.
 * @param ap_index_number The index number of the AP on the host for which to retrieve data.
 * @param allow_ap_transfer_flag Allow for the initiation of a new usb data transfer from the AP.
 *
 * @return APDM_OK if data was retrieved, APDM_NO_MORE_DATA if no more data, error code otherwise.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_next_access_point_record(apdm_ctx_t context, apdm_record_t *data, const int ap_index_number, const bool allow_ap_transfer_flag);

/**
 * Gets the next set of ranging data from the context.
 *
 * @param context
 * @param *data Destination into which to place data.
 *
 * @return APDM_OK if data was retrieved, APDM_NO_MORE_DATA if no more data, error code otherwise.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_next_ranging_record(apdm_ctx_t context, apdm_ranging_sample_t *dest);

/**
 * This function will drop all data stored in the library correlation FIFOs and start reading data from the
 * attached access points until it is able to get a full set of data from all sensors with the same sync value.
 *
 * This function may return failure if one (or more) monitors is catching up it's data stream, you should wait
 * until the AP's are blinking green. If one or more AP's is blinking green-red this function will likely fail.
 *
 * It will also reset the total omitted samples counter.
 *
 * @param context
 *
 * @return APDM_OK if successful, APDM_UNABLE_TO_SYNC_RECORD_HEAD_LIST_ERROR if it can't sync the list due to
 *         lack of data (usually because motion monitors are still docked, or out of range of the access point), error code otherwise
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_sync_record_list_head(apdm_ctx_t context);

/**
 * Queries the APs to determin if they are ready to stream data.
 *
 * @param context
 *
 * @return APDM_OK if successful and ready to stream, APDM_AP_NOT_READY_TO_STREAM if one or more APs are not ready to stream, error code otherwise.
 */
APDM_EXPORT int apdm_ctx_v2_mesh_data_ready(apdm_ctx_t the_context);

/**
 * This function populates an list of records internal to the handle with a set of samples all corresponding
 * to the same sync value (point in time). Depending on the error handling mode set, there may be some samples that
 * are not populated or some partial sample sets that are skipped over.
 *
 * @param context The apdm handle
 *
 * @return APDM_OK If it was able to get a sample set according to the error handling mode, APDM_NO_MORE_DATA if
 *         there is no more data ready (just wait longer for more data to come in), or another code indicating what error occurred.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_next_access_point_record_list(apdm_ctx_t context);




/**
 * This function will purge all but the newest samples from the internal buffers. This is useful in a larger context, when you only
 * want to get the most recent sample(s) for each monitor.
 *
 * E.G.
 * apdm_ctx_populate_buffers()
 * apdm_ctx_purge_older_samples();
 * apdm_ctx_get_next_record2(allow_ap_transfer_flag=false) until it returns no more data, making sure to pass allow_ap_transfer_flag=false
 *
 *
 * @param context The apdm handle
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_purge_older_samples(apdm_ctx_t context);




/**
 * This function is used to flush any data buffers or samples stored in RAM on the AP.
 *
 * @param context The apdm handle
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_flush_ap_fifos(apdm_ctx_t context);

/**
 * Extracts the next single sample from the set of AP's used in the context
 *
 * @param context The apdm handle
 * @param *dest_record The record into which the sample data is to be stored.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_extract_next_sample(apdm_ctx_t context, apdm_record_t *dest_record);

/**
 * This function is used to gather external synchronization I/O data events. For GPIO inputs, input signals are debounced over
 * a 1/2560 second period of time, and the sync value tagged on the synchronization sample will be that of the sync value of the time of the
 * rising edge of the signal. You must be streaming data at the time you call this function, as synchronization events are passed from the
 * AP to the libraries at the time that data is received from the AP.
 *
 * @param context The apdm handle
 * @param *dest Destination into which synchronization data is to be stored.
 *
 * @return APDM_OK if dest was populated with data, APDM_NO_MORE_DATA if there is no synchronization event data available, error code otherwise.
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_next_synchronization_event(apdm_ctx_t context, apdm_external_sync_data_t *dest);

/**
 * This function is used to retrieve button events that have been logged on the Opal. If button events are enabled on the opal, they will be transmited wirelesslessly
 * to the Access point, transfered to the host PC and queued up in the host libraries. This function will retrieve the next pending button event.
 *
 * @param context The apdm handle
 * @param *dest Destination into which button data is to be stored.
 *
 * @return APDM_OK if dest was populated with data, APDM_NO_MORE_DATA if there is no synchronization event data available, error code otherwise.
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_next_button_event(apdm_ctx_t context, apdm_button_data_t *dest);

/**
 * This function will force a transmission of any samples in the AP's from the AP to the host and populate the internal buffers.
 * The internal buffers of the library are where data is temporary stored immediately after a USB transfer. They are used when
 * correlating groups of samples from multiple sensors and as a staging area to store data prior to emission from the libraries.
 *
 * @param context The apdm handle
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_populate_buffers(apdm_ctx_t context);


/**
 * This function will retrieve the oldest sample currently in the library buffers. In the case of multiple samples
 * having the same age, it will return one of the oldest. This function will provide good realtime responsiveness
 * to the caller, however, you may experience duplicates in the data stream or samples coming slightly out of order
 * as samples are emitted as soon as it's available. This function takes a 2-5 milliseconds to execute, so avoid using it
 * in tight data processing loops.
 *
 * Note: this function does not necessarily return a record every time. If a record is available, it will be returned,
 * but depending on timing, wireless conditions, and many other variables, a record may not be available.
 *
 * @param context The apdm handle
 * @param *dest The record into which the data is stored.
 *
 * @return APDM_OK upon success, APDM_NO_MORE_DATA if no data is available, error code otherwise.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_next_record(apdm_ctx_t context, apdm_record_t *dest);


/**
 * Same as apdm_ctx_get_next_record(), except it allows you to disable AP transfers. Note: this function does not necessarily
 * return a record every time. If a record is available, it will be returned, but depending on timing, wireless conditions, and
 * many other variables, a record may not be available.
 *
 * @param context The apdm handle
 * @param *dest The record into which the data is stored.
 * @param allow_ap_transfer_flag Flag to allow you to disable USB transfers from the AP's.
 *
 * @return APDM_OK upon success, APDM_NO_MORE_DATA if no data is available, error code otherwise.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_next_record2(apdm_ctx_t context, apdm_record_t *dest, const bool allow_ap_transfer_flag);


/**
 * @param context The context for which you want the device id
 * @param sensor_index The index of the device id for which you want
 *
 * @return negative error code on error, zero if device is not found at the specified index, device ID greater than 0 on success, only
 *         relevant after auto_configure has been called.
 * @ingroup Context
 */
APDM_EXPORT int32_t apdm_ctx_get_device_id_by_index(apdm_ctx_t context, const uint32_t sensor_index);


/**
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_set_requested_device_states(apdm_ctx_t context, const enum RequestedDeviceState state);

/**
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_set_requested_device_state(apdm_ctx_t context, const enum RequestedDeviceState state, const int ap_index_number);

/**
 * @param ap_handle The access point handle to be configured
 * @param wireless_channel_1 Wireless channel number to use on the first radio
 * @param wireless_channel_2 Wireless channel number to use on the second radio
 * @param device_rx_address_high_order_bytes_A The high-order block ID used for data filtering/matching by the radio (has bit-sequencing constraints for proper hardware behavior)
 * @param device_rx_address_high_order_bytes_B The high-order block ID used for data filtering/matching by the radio (has bit-sequencing constraints for proper hardware behavior)
 * @param radio1_pipe_count Number of pipes to enable
 * @param radio2_pipe_count Number of pipes to enable
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_init_access_point_wireless(apdm_ap_handle_t ap_handle,
		const uint8_t wireless_channel_1, const uint8_t wireless_channel_2,
		const uint32_t device_rx_address_high_order_bytes_A,
		const uint32_t device_rx_address_high_order_bytes_B,
		const uint8_t radio1_pipe_count, const uint8_t radio2_pipe_count);

/**
 * @param context The context for which you want the device id
 * @param id The motion monitor ID for which you want the index of.
 * @param *dest_index The index of the specified device id
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_device_index_by_id3(apdm_ctx_t context, const uint32_t id, uint32_t *dest_index);


/**
 * @param context The context for which you want the device id
 * @param *dest Destination array of uint32_t's into which you want to store devices IDs, the (last+1) element will have a device id of zero.
 * @param destSize The number of elements in the destination array.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_device_id_list(apdm_ctx_t context, uint32_t *dest, const uint32_t destSize);

/**
 * Internal function to configure a single access point with a given pipe count and assign wireless channels
 * based on whats configured in the AP handle data structure.
 *
 * @param ap_handle The handle for the AP to be configured.
 * @param radio1_pipe_count The number of pipes that will be used for radio 1 of the AP (first three devices usually)
 * @param radio2_pipe_count The number of pipes that will be used for radio 2 of the AP (first three devices usually)
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_configure_accesspoint(apdm_ap_handle_t ap_handle, const uint8_t radio1_pipe_count, const uint8_t radio2_pipe_count);

/**
 * The access point tracks some internal debugging stats and numbers. This function will retrieve
 * those debugging statistic and print to the debug logging subsystem.
 *
 * @param context
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup AccessPoint
 */
APDM_EXPORT int apdm_ctx_get_all_ap_debug_info(apdm_ctx_t context);



//Device L2


/**
 * Gets the Module ID that the dock things is currently placed in the dock.
 *
 * @param device_handle The docking station handle
 * @param *dest Destination into which you want the module stored into.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_get_docked_module_id(apdm_device_handle_t device_handle, uint32_t *dest);

/**
 * @param device_handle The device handle
 * @param *dest_protocol_subversion The destination into which to store the protocol version
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_get_protocol_subversion(apdm_device_handle_t device_handle, int64_t *dest_protocol_subversion);

/**
 * Note this only works in firmware/bootloaders after Nov 8, 2010
 *
 * @param device_handle The docking station handle
 * @param *dest The destination into which to store the hardware revision number of the dock
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_get_hardware_version(apdm_device_handle_t device_handle, uint32_t *dest);

/**
 * Note only works in firmware/bootloaders after Nov 8, 2010
 *
 * @param device_handle The docking station handle
 * @param *dest The destination into which to store the dock firmware version
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_get_firmware_version(apdm_device_handle_t device_handle, uint64_t *dest);

/**
 *
 * @param device_handle The docking station handle
 * @param *BYTE_ARRAY Destination into which to store the Case ID String
 * @param dest_buffer_length The length of the buffer to which dest_buffer is pointing
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_get_case_id(apdm_device_handle_t device_handle, char *BYTE_ARRAY, const int dest_buffer_length);



/**
 * Used to retrieve the serial number of a given docking station index number.
 *
 * @param docking_station_index Index of the docking station for which you want the serial number
 * @param *serial_number Destination into which the serial number will be stored
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_get_serial_number_by_index(const int docking_station_index, uint32_t *serial_number);

/**
 * @param device_handle The docking station handle
 * @param *output_flag The destination into which to store the indicator as to weather or not there is an monitor present in
 *                     the dock, zero indicates dock is empty, non-zero indicates a monitor is present.
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_is_monitor_present(apdm_device_handle_t device_handle, uint32_t *output_flag);

/**
 * @param device_handle The device handle
 * @param *output_flag Destination into which to store the current status of weather or not data forwarding is enabled, zero indicates
 *                     data is not being forwarded, non-zero indicates it is being forwarded
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_is_monitor_data_forwarding_enabled(apdm_device_handle_t device_handle, uint32_t *output_flag);

/**
 * @param device_handle The docking station handle for which you want the serial number
 * @param *serial_number Destination into which to store the dock serial number
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_get_serial(apdm_device_handle_t device_handle, uint32_t *serial_number);

/**
 * This will return the index of the of the docking station with with the specified serial number.
 *
 * @param serial_number The serial number of the docking station for which you want the index of.
 * @param *docking_station_index The destination into which you want the index to be stored.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup DockingStation
 *
 */
APDM_EXPORT int apdm_ds_get_index_by_serial_number(const uint32_t serial_number, uint32_t *docking_station_index);


/**
 *
 * @param ds_handle The handle to the device to apply the configuration to.
 * @param baud_mode 0 to disable high speed baud rates (default), 57600 to enable 57600 baud rate auto negotiation
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup DockingStation
 */
APDM_EXPORT int apdm_ds_set_monitor_baud_rate(apdm_device_handle_t ds_handle, const uint32_t baud_mode);


/**
 * Applys default configuration settings to a apdm_device_info_t structure
 *
 * @param *device_info Pointer to the structure to be initialized
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_initialize_device_info(apdm_device_info_t *device_info);

/**
 * FIXME document this
 */
APDM_EXPORT int apdm_initialize_offset_test_results(apdm_device_status_t *results);

/**
 * FIXME document this
 */
APDM_EXPORT int apdm_sensor_test_offsets(apdm_device_handle_t device_handle, apdm_device_info_t *device_info, apdm_device_status_t *results);

/**
 * @param device_handle The handle to the device to apply the configuration to.
 * @param device_info The configuration to be applied to the device.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_apply_configuration(apdm_device_handle_t device_handle, apdm_device_info_t *device_info);




/**
 * Extracts the module ID from a monitor case ID string, such as "SI-000025" will find 25.
 *
 * @param *case_id Case ID string from the motion monitor
 * @param *dest_module_id Destination into which to store the module ID
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_device_extract_module_id_from_case_id_string(const char *case_id, uint32_t *dest_module_id);

/**
 * This function will be removed after Jan 2014.
 *
 * Retrieves a list of device ID's attached to the host.
 *
 * @param *serial_number_buffer Pointer to an array of uint32_t into which the serial numbers should be stored.
 * @param buffer_length The number of elements in the destination array.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_DEPRECATED APDM_EXPORT int apdm_sensor_get_device_id_list(uint32_t *serial_number_buffer, const uint32_t buffer_length);


/**
 * Parses a case ID string from a motion monitor and identifies which type of monitor it is (opal, emerald, saphire)
 *
 * @param case_id_string Case ID from the monitor
 * @param *dest Destination into which the monitor type will be stored
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_get_monitor_type(const char *case_id_string, apdm_monitor_type_t *dest);





/**
 * Halts all sensors that are connected to the host. Note, make sure all device handles and contexts have been closed prior to calling this function.
 *
 * @return Zero on success, non-zero error code if failure.
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_halt_all_attached_sensors(void);

/**
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_configure_wireless(apdm_device_handle_t device_handle, const enum APDMDeviceConfig wirelessConfigType, const uint32_t value);

/**
 * When the motion monitor is removed from the dock, or disconnected from the cable, the motion monitor will halt.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */

APDM_EXPORT int apdm_sensor_cmd_halt(apdm_device_handle_t device_handle);

/**
 * When the motion monitor has been commanded to halt, you can un-set the halt flag on the monitor.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_resume(apdm_device_handle_t device_handle);

/**
 * @param device_handle The device handle.
 * @param *dest Destination into which to put the device info associated with the device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Monitor
 */
APDM_EXPORT int apdm_sensor_populate_device_info(apdm_device_handle_t device_handle, apdm_device_info_t *dest);


/**
 * FIXME document this
 */
APDM_EXPORT int apdm_sensor_get_megabytes_total(apdm_device_handle_t sensor_handle, uint32_t *dest);

/**
 * FIXME document this
 */
APDM_EXPORT int apdm_sensor_get_megabytes_used(apdm_device_handle_t sensor_handle, uint32_t *dest);

/**
 * Causes the monitor to reset.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_reset(apdm_device_handle_t device_handle);

/**
 * Commands the device to enter run mode.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_run(apdm_device_handle_t device_handle);

/**
 * Does a CRC check on the given address and length of flash in the motion monitor
 *
 * @param device_handle The device handle.
 * @param address The start address of the CRC check.
 * @param length The number of bytes to CRC
 * @param *current_value Destination into which to store the CRC value.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_memory_crc16(apdm_device_handle_t device_handle, const uint32_t address, const uint16_t length, uint16_t *current_value);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_write_flash_block(apdm_device_handle_t device_handle, const uint32_t address, uint8_t *data, const uint32_t length);

/**
 * Sets the time on the motion monitor
 * @param device_handle The device handle.
 * @param year 0-9999
 * @param month 1-12
 * @param day 1-31
 * @param hour 0-23
 * @param minute 0-59
 * @param second 0-59
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_time_set(apdm_device_handle_t device_handle, uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second);

/**
  * Sets the time on the device to the current host computer's time
  * Only compatible with v2 sensors
  * @param device_handle The device handle.
  */
APDM_EXPORT int apdm_set_time_now(apdm_device_handle_t device_handle);

/**
 * FIXME document this
 */
APDM_EXPORT int apdm_sensor_cmd_timezone_set(apdm_device_handle_t device_handle, const uint32_t timezone_offset);

/**
 * Sets the time on the Motion Monitor in terms of the epoch time (number of seconds since 1970)
 *
 * @param device_handle The device handle.
 * @param epoch_time Number of seconds since 1970.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_time_set2(apdm_device_handle_t device_handle, const time_t epoch_time);

/**
 * Retrieves the time from the motion monitor.
 *
 * @param device_handle The device handle.
 * @param *year
 * @param *month
 * @param *day
 * @param *hour
 * @param *minute
 * @param *second
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_time_get(apdm_device_handle_t device_handle, uint32_t *year, uint32_t *month, uint32_t *day, uint32_t *hour, uint32_t *minute, uint32_t *second);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_flash_block_set(apdm_device_handle_t device_handle, uint32_t block);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_flash_block_get(apdm_device_handle_t device_handle, uint32_t *block);

/**
 * @param device_handle The device handle.
 * @param *voltage Destination into which to store the battery voltage, this is in units of the raw ADC value off the MCU.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_battery_voltage(apdm_device_handle_t device_handle, uint16_t *voltage);

/**
 * Sets the battery charge rate
 *
 * @param device_handle The device handle.
 * @param rate Units of milliamps, minimum = 100mA, max = 450mA;
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_battery_charge_rate(apdm_device_handle_t device_handle, uint16_t rate);

/**
 * Gets the version of calibration date currently on the motion monitor
 *
 * @param device_handle The device handle.
 * @param *calibration_version Destination into which to store the calibration varsion number
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_calibration_version(apdm_device_handle_t device_handle, uint32_t *calibration_version);

/**
 * Configures the device to optionally use the calculated calibration data, as opposed to factory defaults
 * 
 * @param device_handle The device handle.
 * @param enable Whether to use calibration data when computing sensor data (true) or not (false)
 *
 * @return APDM_OK on success, orr code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_use_calibration(apdm_device_handle_t device_handle, bool useCalibration);

/**
 * @param device_handle The device handle.
 * @param monitor_memory_address The start address in the monitors address space to start reading from
 * @param num_bytes_to_read The number of bytes of memory to read from the device, must be greater than 0 and less then 32768
 * @param *BYTE_ARRAY The destination into which the memory dump is to be stored, must be non-null
 * @param dest_buffer_length The length of the destination buffer pointed to by *BYTE_ARRAY, must be greater than 0 and greater than or equal to num_bytes_to_read
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_memory_dump(apdm_device_handle_t device_handle, const uint32_t monitor_memory_address, const int num_bytes_to_read, char *BYTE_ARRAY, const int dest_buffer_length);

/**
 * Peeks an 8-bit value in the motion monitor address space.
 *
 * @param device_handle The device handle.
 * @param address The address to be peeked
 * @param *current_value The destination pointer into which to store the value.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_peek(apdm_device_handle_t device_handle, const uint32_t address, uint8_t *current_value);

/**
 * Peeks an 16-bit value in the motion monitor address space.
 *
 * @param device_handle The device handle.
 * @param address The address to be peeked
 * @param *current_value The destination pointer into which to store the value.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_peek2(apdm_device_handle_t device_handle, const uint32_t address, uint16_t *current_value);

/**
 * Writes an 8-bit value into the address space of the motion monitor, note, writing to flash address space won't work.
 *
 * @param device_handle The device handle.
 * @param address The address at which to write to
 * @param new_value The value to be written
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_poke(apdm_device_handle_t device_handle, const uint32_t address, uint8_t new_value);

/**
 * Writes an 16-bit value into the address space of the motion monitor, note, writing to flash address space won't work.
 *
 * @param device_handle The device handle.
 * @param address The address at which to write to
 * @param new_value The value to be written
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_poke2(apdm_device_handle_t device_handle, const uint32_t address, uint16_t new_value);

/**
 * Retrieves the sync value currently on the motion monitor.
 *
 * @param device_handle The device handle.
 * @param *current_value The destination into which to store the current sync value on the motion monitor
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_sync_get(apdm_device_handle_t device_handle, uint64_t *current_value);

/**
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_sync_dock_wait(apdm_device_handle_t device_handle);

/**
 * Led pattern is sent to the device as a character string which represents the led color pattern to display.
 *
 * @param device_handle The device handle.
 * @param *pattern
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_led_pattern(apdm_device_handle_t device_handle, uint8_t interval, uint8_t *pattern, uint8_t length);

/**
 * Tells the device to go back to its normal led sequence (after having been overriden by apdm_sensor_cmd_led_pattern())
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_led_reset(apdm_device_handle_t device_handle);

/**
 * @param device_handle The device handle.
 * @param *reason Destination into which to store the reason code, reason is defined in 'enum apdm_monitor_off_reason_t' in apdm_types.h
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_off_reason(apdm_device_handle_t device_handle, uint8_t *reason);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_uptime_get(apdm_device_handle_t device_handle, uint32_t *uptime);

/**
 * This command resets the uptime counter on the device.  Mainly useful as a debugging command.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_uptime_reset(apdm_device_handle_t device_handle);

/**
 * This command returns the last max uptime the device achieved while running before powering off or going into standby mode. Mainly useful as a debugging command.
 *
 * @param device_handle The device handle.
 * @param *uptime Destination into which to store the last uptime.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */

APDM_EXPORT int apdm_sensor_cmd_last_uptime(apdm_device_handle_t device_handle, uint32_t *uptime);

/**
 * This command returns the last max uptime the device achieved while in standby mode. Mainly useful as a debugging command.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_last_standby_uptime(apdm_device_handle_t device_handle, uint32_t *uptime);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_unlock_bootloader_flash(apdm_device_handle_t device_handle);

/**
 * @param device_handle The device handle.
 * @param password The password, of length 8ap_handle_version, to enter the bootloader (password differs based on monitor version)
 * @param password_length The length of the password, must be 8 bytes long.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_enter_bootloader(apdm_device_handle_t device_handle, const char *password, const int password_length);

/**
 * This command is only supported on v1.1 or later monitors. Previous monitor version will result in a return code of APDM_DEVICE_RESPONSE_ERROR_INVALID_COMMAND.
 *
 * @param device_handle The device handle.
 * @param *dest_version The destination into which to store the bootloader version number
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_bootloader_version(apdm_device_handle_t device_handle, uint32_t *dest_version);

/**
 * Starts a 1-second cycle of the device sampling data on its internal sensors.
 * Using the following settings:
 * - output rate 128
 * - decimation factor 5x2
 * - no mag set/reset
 * - all sensors enabled
 * - temperature from gyro
 *
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_sample_start(apdm_device_handle_t device_handle);

/**
 * Gets the 1 second of data that was initiated by the sample_start() command.
 *
 * The format of the data returned is a set of 4x 512byte blocks with 25
 * sample sets each.  There will be 12bytes of padding at the end of each
 * 512byte block.  This provides the host with 100 total samples.  Each
 * sample is a 16bit value with a sample set packed in the following order
 * AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,T. (T=temperature)
 *
 *
 * @param device_handle The device handle.
 * @param *dest_buffer Destination buffer into which to store samples taken.
 * @param buff_length The length of the buffer pointed to by *dest_buffer, must be greater than or equalt to 2048 bytes
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_sample_get(apdm_device_handle_t device_handle, uint8_t *dest_buffer, const int buff_length);

/**
 * Sets the sync value on the motion monitor, should call cmd_sync_commit() sometime after the sync value is set.
 *
 * @param device_handle The device handle.
 * @param new_value The new sync value to be set.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_sync_set(apdm_device_handle_t device_handle, const uint64_t new_value);

/**
 * Commits the sync value previously set by cmd_sync_set() thus causing the change to take effect.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_sync_commit(apdm_device_handle_t device_handle);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_config_commit(apdm_device_handle_t device_handle);

/**
 * This command queries if the device is present and what its state is in regards to the bootloader (pre-bootloader/bootloader/post-bootloader).
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_ping(apdm_device_handle_t device_handle, uint8_t *mode);

/**
 * Retrieves the device ID off the motion monitor
 *
 * @param device_handle The device handle.
 * @param *current_value Destination into which to store the device id.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_device_id(apdm_device_handle_t device_handle, uint32_t *current_value);

/**
 * Gets the number of errors on the motion monitor.
 *
 * @param device_handle The device handle.
 * @param *error_count Destination into which to store the error count.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_error_count(apdm_device_handle_t device_handle, uint32_t *error_count);

/**
 * Retrieves the name of the specified error ID.
 *
 * @param device_handle The device handle.
 * @param *BYTE_ARRAY Destination string into which to store the name of the error
 * @param length The size of the *BYTE_ARRAY string buffer
 * @param error_id The ID for which you want to retrieve the error name
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_error_name(apdm_device_handle_t device_handle, char *BYTE_ARRAY, const int length, uint16_t error_id);

/**
 * Retrieves the size of the error log on the motion monitor
 *
 * @param device_handle The device handle.
 * @param *error_log_size Destination into which to store the error log size.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_error_log_size(apdm_device_handle_t device_handle, uint16_t *error_log_size);

/**
 * Retrieves the number of times the error at offset has occurred.
 *
 * @param device_handle The device handle.
 * @param offset The error number offset to retrieve
 * @param *error_id Destination into which to store the error count for the specified offset.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_error_log_get(apdm_device_handle_t device_handle, const uint16_t offset, uint16_t *error_id);


/**
 * @param device_handle The device handle.
 * @param v2_state_id ID of the state to query, from monitor_state_t
 * @param *dest Store 0 into dest if the error is not currently asserted, 1 otherwise
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_state_status(apdm_device_handle_t device_handle, const uint32_t v2_state_id, uint16_t *dest);

/**
 * @param device_handle The device handle.
 * @param *stats_size FIXME document this
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_error_stats_size(apdm_device_handle_t device_handle, uint16_t *stats_size);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_error_stats_get(apdm_device_handle_t device_handle, const uint16_t id, uint16_t *count);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_stats_size(apdm_device_handle_t device_handle, uint16_t *value);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_stats_max_get(apdm_device_handle_t device_handle, const uint16_t id, uint16_t *max_val);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_stats_min_get(apdm_device_handle_t device_handle, const uint16_t id, uint16_t *min_val);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_stats_count_get(apdm_device_handle_t device_handle, const uint16_t id, uint16_t *count_val);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_stats_sum_get(apdm_device_handle_t device_handle, const uint16_t id, uint32_t *sum_val);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_stats_clear(apdm_device_handle_t device_handle);

/**
 * Clears all errors and error stats on the motion monitor.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_error_clear(apdm_device_handle_t device_handle);

/**
 * Retrieves the battery charge status
 *
 * @param device_handle The device handle.
 * @param *current_status Destination into which to store the battery charge status, values are defined in "enum APDM_Battery_Charge_Status" in apdm_types.h.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_battery_charge_status(apdm_device_handle_t device_handle, uint8_t *current_status);


/**
 * Returns the packed binary representation of the motion monitor calibration data
 *
 * @param device_handle The device handle.
 * @param dest Destination buffer into which to store the packed cal data.
 * @param dest_length The size of the destination buffer.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_calibration_data_blob(apdm_device_handle_t device_handle, uint8_t *dest, const int dest_length);

/**
 * Retrieves the user calibration data from the sensor (from re-calibration in the field)
 *
 * @param dev_handle The device handle of which you want user calibration from.
 * @param dest The destination buffer into which binary data is to be stored.
 * @param dest_length The length of the destination buffer, which is not to be overrun
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_user_calibration_data_blob(apdm_device_handle_t dev_handle, uint8_t *dest, const int dest_length);

/**
 * This function will retrieve the sensor calibration data from the given devices via dev_handle
 *
 * @param device_handle The device handle.
 * @param *sensor_comp Destination into which to store sensor compensation data
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_calibration_data(apdm_device_handle_t device_handle, apdm_sensor_compensation_t *sensor_comp);

/**
 * This function will retrieve the user-overridden sensor calibration data from the given devices via dev_handle.
 * Requires monitor firmware versions newer then March 2011.
 *
 * @param dev_handle The device handle.
 * @param *sensor_comp Destination into which to store sensor compensation data
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_user_calibration_data(apdm_device_handle_t dev_handle, apdm_sensor_compensation_t *sensor_comp);

/**
 * @param device_handle The device handle.
 * @param *BYTE_ARRAY Destination into which to store the version string.
 * @param dest_buff_length length of the *BYTE_ARRAY array.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_version_string_1(apdm_device_handle_t device_handle, char *BYTE_ARRAY, const int dest_buff_length);

/**
 *
 * @param device_handle The device handle.
 * @param *BYTE_ARRAY Destination into which to store the version string.
 * @param dest_buff_length length of the *BYTE_ARRAY array.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_version_string_2(apdm_device_handle_t device_handle, char *BYTE_ARRAY, const int dest_buff_length);

/**
 * @param device_handle The device handle.
 * @param *BYTE_ARRAY Destination into which to store the version string.
 * @param dest_buff_length length of the *BYTE_ARRAY array.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_version_string_3(apdm_device_handle_t device_handle, char *BYTE_ARRAY, const int dest_buff_length);

/**
 * @param device_handle The device handle.
 * @param *status Values defined in enum apdm_monitor_dock_status_t
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_dock_status(apdm_device_handle_t device_handle, uint8_t *status);

/**
 * Retrieves a specified configuration parameter type. For V2 hardware, see apdm_sensor_config_get_v2().
 *
 * @param device_handle The device handle.
 * @param config_type The configuration parameter type.
 * @param *value The destination into which to store the parameter value.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_config_get(apdm_device_handle_t device_handle, const enum APDMDeviceConfig config_type, uint32_t *value);


/**
 * Sets a specified configuration parameter. For V2 hardware, see apdm_sensor_config_set_v2().
 *
 * @param device_handle The device handle.
 * @param config_type The parameter which is to be set, from enum APDMDeviceConfig in apdm_types.h.
 * @param value The value to which it is to be set. There are enums in apdm_types.h to specify valid values for various parameter types.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_config_set(apdm_device_handle_t device_handle, const enum APDMDeviceConfig config_type, const uint32_t value);

/**
 * Clear all logged data from the device
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_clear_logged_data(apdm_device_handle_t device_handle);

/**
 * Wrapper function for setting the label config parameter
 *
 * @param device_handle The device handle.
 * @param label_str 16 character array source for label.
 * @param str_length The length of the buffer pointed to by label_str
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_config_set_label(apdm_device_handle_t device_handle, const char label_str[16], const int str_length);

/**
 * Specific to v2 device handles. Allows of setting of individual config settings on the device.
 *
 * @param device_handle The device handle.
 * @param setting From the enumeration config_value_t, specifies what setting is to be changed.
 * @param value The new value for that setting
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_config_set_v2(apdm_device_handle_t h, const config_value_t setting, const uint32_t value);

/**
 * Specific to v2 device handles. Retrieves the config setting from the given device.
 *
 * @param device_handle The device handle.
 * @param setting From the enumeration config_value_t, specifies what setting is to be retrieved.
 * @param *dest Destination pointer into which to store the current setting.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_config_get_v2(apdm_device_handle_t h, const config_value_t setting, uint32_t *dest);

/**
 * Specific to v2 device handles. This is not supported on V1 device handles.
 *
 * @param device_handle The device handle.
 * @param label_string The label to be set.
 * @param label_str 24 character array source for label.
 * @param str_length The length of the buffer pointed to by label_str
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_config_set_label_v2(apdm_device_handle_t h, const config_v2_string_t label_string, const char label_str[24], const int str_length);

/**
 * Wrapper function for getting the label config parameter
 *
 * @param device_handle The device handle.
 * @param BYTE_ARRAY 16 character array destination to store the label.
 * @param buff_size The size of the buffer into which to store the label, must be at least 16
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_config_get_label(apdm_device_handle_t device_handle, char *BYTE_ARRAY, const int buff_size);

/**
 * Wrapper function for getting the label config parameter
 *
 * @param device_handle The device handle.
 * @param label_string The label to be set.
 * @param BYTE_ARRAY 24 character array destination to store the label.
 * @param buff_size The size of the buffer into which to store the label, must be at least 16
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_config_get_label_v2(apdm_device_handle_t device_handle, const config_v2_string_t label_string, char *BYTE_ARRAY, const int buff_size);

/**
 * This command returns the current status of if the configuration has been committed or not.
 *
 * @param device_handle The device handle.
 * @param *status Destination into which to put the configuration status, 0 indicates the config has not been committed, 1 indicates that it has been committed.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_config_status(apdm_device_handle_t device_handle, uint8_t *status);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_timer_adjust_get(apdm_device_handle_t device_handle, uint16_t *value);

/**
 * This command is used to instruct the device to into run mode.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_run(apdm_device_handle_t device_handle);

/**
 * This command sets the debug value identified by the id parameter.
 * @param device_handle The device handle.
 * @param id The debug variable ID which is to be set
 * @param data The value to which it is to be set.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_debug_set(apdm_device_handle_t device_handle, uint8_t id, uint32_t data);

/**
 * This command gets the debug value identified by the id parameter.
 * @param device_handle The device handle.
 * @param id The debug variable ID which is to be retrieved
 * @param data The destination into which to store the debug value.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_debug_get(apdm_device_handle_t device_handle, uint8_t id, uint32_t *data);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_dock(apdm_device_handle_t device_handle);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_undock(apdm_device_handle_t device_handle);

/**
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * Sets what is pointed to by is_valid to 1 if the configuration is valid, sets it to 0 otherwise.
 *
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_config_check(apdm_device_handle_t device_handle, uint8_t *is_valid);

/**
 * Causes the motion monitor to re-format it's SD card when it is removed from the docking station or device cable.
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_flash_format(apdm_device_handle_t device_handle);

/**
 * Standby mode on the opal allows the device to retain the correct time while not recording.  Issuing the standby
 * command to the opal will instruct it to transition to this mode the next time it is undocked. The device will
 * appear to power off but will instead be in a low power state updating the clock once a second.  The duration that
 * the device can stay in this mode before needing to fully power off will be dependent on how much battery charge
 * is available.  This allows users to ship or otherwise store the device for between a day to a week while keeping
 * the time correct on the device.
 *
 *
 * @param device_handle The device handle.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_standby(apdm_device_handle_t device_handle);

/**
 * Retrieves the case ID from the motion monitor
 *
 * @param device_handle The device handle.
 * @param *BYTE_ARRAY Destination buffer into which to store the case ID
 * @param dest_buff_length size of *BYTE_ARRAY
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_case_id(apdm_device_handle_t device_handle, char *BYTE_ARRAY, const int dest_buff_length);

/**
 * Retrieves the hardware ID of the motion monitor.
 *
 * @param device_handle The device handle.
 * @param *current_value The destination into which to store the hardware ID.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup MonitorCommands
 */
APDM_EXPORT int apdm_sensor_cmd_hw_id(apdm_device_handle_t device_handle, uint32_t *current_value);

//L3


/**
 * Used to initialize a handle context
 *
 * @param context The handle to be initialized
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_initialize_context(apdm_ctx_t context);







/**
 * Disconnects from access points that are currently attached (USB bus handle disconnect)
 *
 * @param context The handle to be disconnected
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_disconnect(apdm_ctx_t context);


/**
 * This function clears out any kernel event handlers or callbacks.
 * Before unloading the DLL/SO/DYLIB and program termination, this function should be called.
 * This function should be called just before you program exits (do not call this function
 * if you intended to continue using the APDM library, wait until your completely done.)
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_exit(void);

/**
 * Allocates memory a handle to be used by the apdm libraries.
 *
 * @return Non-zero on success, zero otherwise
 * @ingroup Context
 */
APDM_EXPORT apdm_ctx_t apdm_ctx_allocate_new_context(void);

/**
 * De-allocates memory used for the APDM handle context
 *
 * @param context The handle to be deallocated.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_free_context(apdm_ctx_t context);


/**
 * This function will take a context, and persist all the configuration information to disk thus
 * allowing it to be restored at a later time with apdm_ctx_restore_context_from_disk() and avoid the need
 * to re-configure the system.
 *
 * @param context The handle to be deallocated.
 * @param filepath The file to save the context to
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_persist_context_to_disk(apdm_ctx_t context, const char *filepath);

/**
 * Restore a context from disk to memory thus allowing you to start streaming without re-configuring the
 * system. Make sure to re-sync the record head list prior to streaming data.
 *
 * This functionality will only work if identical access points are used, and all of the original
 * monitors are in use.
 *
 * @param context Destination context into which to restore context information.
 * @param filepath The file to restore the context from.
 * @return APDM_OK on success, APDM_UNEXPECTED_STRUCTURE_VALUE if the version of the libraries that saved
 *         the file is different then the version of libraries that is restoring the context, error code otherwise.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_restore_context_from_disk(apdm_ctx_t context, const char *filepath);


/**
 * Same as apdm_autoconfigure_devices_and_accesspoint4(), but extra parameter allows you to set the decimation rate.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_autoconfigure_devices_and_accesspoint5(apdm_ctx_t context, const uint8_t wireless_channel_number, const bool enable_sd_card,
		const bool erase_sd_card, const bool accel_full_scale_mode, const bool enable_accel, const bool enable_gyro, const bool enable_mag, const apdm_monitor_decimation_rate_t decimation_rate);

/**
 * Initializes streaming_configuration data structure to default values.
 *
 * wireless_channel_number = 80;
 * enable_sd_card = true;
 * erase_sd_card = false;
 * accel_full_scale_mode = true;
 * enable_accel = true;
 * enable_gyro = true;
 * enable_mag = true;
 * apply_new_sensor_modes = true;
 * decimation_rate = APDM_DECIMATE_5x2;
 * output_rate_hz = 128;
 * button_enable = false;
 *
 * @param *streaming_config Pointer to apdm_streaming_config_t structure to be initialized
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_init_streaming_config(apdm_streaming_config_t *streaming_config);

/**
 * Helper method to retreive a reference to an apdm_device_info_t structure.
 * Useful for the Java SWIG binding.
 *
 * @param *streaming_config Pointer to apdm_streaming_config_t structure to be used
 * @param sensor_index The index into the array of apdm_device_info_t structures
 *
 * @return Pointer to the corresponding apdm_device_info_t structure.
 * @ingroup Misc
*/
APDM_EXPORT apdm_device_info_t* apdm_streaming_config_get_device_info(apdm_streaming_config_t *streaming_config, int sensor_index);

/**
 * Used to autoconfigure accesspoints and sensors based on contents of apdm_streaming_config_t data structure, replacement for the
 * numeric variations of apdm_autoconfigure_devices_and_accesspoint###() functions.
 *
 * @param context
 * @param  streaming_config The configuration to be applied to the system.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_autoconfigure_devices_and_accesspoint_streaming(apdm_ctx_t context, apdm_streaming_config_t *streaming_config);

/**
 * When apdm_autoconfigure_devices_and_accesspoint_streaming() is called with set_configuration_on_device set to false, this function can be called after the fact
 * to apply the configuration of the monitor to the respective monitor that is on the docking station.
 *
 * @param context
 * @param ds_handle A docking handle, that has been opened, and has a monitor present in it.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_apply_autoconfigure_sensor_config(apdm_ctx_t context, apdm_device_handle_t ds_handle);

/**
 * This function is similar to apdm_autoconfigure_devices_and_accesspoint4(), except that it doesn't override whatever device
 * settings are already present on the attached devices.
 *
 * @param context
 * @param wireless_channel_number The base wireless channel to transmit data on, 0-100.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Setup
 */
APDM_EXPORT int apdm_autoconfigure_devices_and_accesspoint_wireless(apdm_ctx_t context, const uint8_t wireless_channel_number);



/**
 * This function is used to configure all Motion Monitors currently attached to the host in synchronized logging mode, maximum of 32 devices.
 *
 * @param context
 * @param wireless_channel_number The wireless channel used to synchronize time between the motion monitors in the mesh time sync group.
 * @param enable_sd_card Boolean indicating weather or not data should be logged to the SD card on the device.
 * @param erase_sd_card Boolean flag indicating that the data on the SD card should be erased as part of the initialization process.
 * @param accel_full_scale_mode If true, then accelerometers will be in 6G mode, if false, then they will be in 2G mode
 * @param enable_accel Enable the accelerometers
 * @param enable_gyro Enable the gyros
 * @param enable_mag Enable the magnitometers
 *
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_autoconfigure_mesh_sync(apdm_ctx_t context, const uint8_t wireless_channel_number,
		const bool enable_sd_card, const bool erase_sd_card,
		const bool accel_full_scale_mode, const bool enable_accel, const bool enable_gyro, const bool enable_mag);

/**
 * This function is used to configure all Motion Monitors currently attached to the host in synchronized logging mode, maximum of 32 devices.
 *
 * @param context
 * @param *streaming_config Pointer to config structure with all the settings for the Opals.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_ctx_autoconfigure_devices_mesh(apdm_ctx_t context, apdm_streaming_config_t *streaming_config);

/**
 * Similar to apdm_autoconfigure_mesh_sync(), however this will configure all attached monitors into synchronized logging mode without modifying
 * the pre-existing sensor settings on the monitors.
 *
 * @param context
 * @param wireless_channel_number The wireless channel used to synchronize time between the motion monitors in the mesh time sync group.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_autoconfigure_mesh_sync2(apdm_ctx_t context, const uint8_t wireless_channel_number);


/**
 * This function will disable the wireless radios and protocol on all the access points in the context, causing them to no longer
 * transmit sync packets, nor be able to RX data from monitors.
 *
 * @param context
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_disable_accesspoint_wireless(apdm_ctx_t context);

/**
 * After wireless has been disabled on an AP using the apdm_ctx_disable_accesspoint_wireless() function, in can be re-enabled using this function
 *
 * @param context
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_re_enable_accesspoint_wireless(apdm_ctx_t context);

/** This function will automatically configure all attached access points and devices in such a way that data can be streamed from the the system.
 *
 * @param context
 * @param enable_sd_card Boolean indicating weather or not data should be logged to the SD card on the device.
 * @param erase_sd_card Boolean flag indicating that the data on the SD card should be erased as part of the initialization process.
 * @param accel_full_scale_mode If true, then accelerometers will be in 6G mode, if false, then they will be in 2G mode
 * @param enable_accel Enable the accelerometers
 * @param enable_gyro Enable the gyros
 * @param enable_mag Enable the magnitometers
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Setup
 */
APDM_EXPORT int apdm_configure_all_attached_sensors(apdm_ctx_t context, const bool enable_sd_card, const bool erase_sd_card,
		const bool accel_full_scale_mode, const bool enable_accel, const bool enable_gyro, const bool enable_mag);

/**
 * Checks to see if more data is available in the host-resident sample buffers and if a subsequent call to get data would return without doing
 * a USB bus transfer
 *
 * @param context The apdm context to check data on
 *
 * @return Zero if there is no more data, non-zero if there is more data available.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_is_more_data_immediately_available(apdm_ctx_t context);

/**
 *  Returns the average number of retries for samples coming from the given device, useful as a wireless reliability indicator
 *  for the device, (only accurate while actively streaming data thru the host libraries).
 *
 *  @param context
 *  @param device_id The device ID (this is different then the Case ID on the back of the monitor)
 *
 *  @return Negative error code on error, zero or higher with average number of retries per second
 *           for device id specified over previous 3 seconds.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_avg_retry_count_for_device(apdm_ctx_t context, const uint32_t device_id);

/**
 * @param context
 * @param sensor_index The index of the given sensor for which you want error stats
 * @param error_id The ID, per apdm_motion_monitor_error_id_t, of the error that you want
 * @param *dest The destination into which you want the stats to be stored.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 *
 * @relates apdm_motion_monitor_error_id_str()
 */
APDM_EXPORT int apdm_ctx_get_error_stats(apdm_ctx_t context, const uint32_t sensor_index, const int error_id, apdm_monitor_error_stat_t *dest);

/**
 * @param error_id The error ID, of type, apdm_motion_monitor_error_id_t, for which you want a string representation
 *
 * @return Const char* pointing to string representation of the given error ID.
 * @ingroup Misc
 */
APDM_EXPORT const char* apdm_monitor_error_id_str(const apdm_monitor_error_id_t error_id);

/**
 * Used to get a number between 0 and 100 on how reliable the wireless connection is for a given device, (only accurate while
 * actively streaming data thru the host libraries).
 *
 * @param context
 * @param device_id The device ID (this is different then the Case ID on the back of the monitor)
 *
 * @return Negative error code on error, Zero to 100 on success, 100 being the best signal, zero being the worst (or no).
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_wireless_reliability_value(apdm_ctx_t context, const uint32_t device_id);

/**
 * @param context
 * @param device_id The device ID (this is different then the Case ID on the back of the monitor)
 * @param *dest Destination into which to store the RSSI value for the given device_id.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_rssi_value(apdm_ctx_t context, const uint32_t device_id, uint32_t *dest);

/**
 * @param context
 * @param *dest Destination into which to store the wireless streaming status by AP, of type apdm_ap_wireless_streaming_status_t.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_wireless_streaming_status(apdm_ctx_t context, uint32_t *dest);

/**
 * @param context
 * @param *dest Destination into which to store the streaming status by AP.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 *
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_streaming_status(apdm_ctx_t context, const uint32_t device_id, apdm_streaming_status_t *dest);

/**
 * Gets the unix epoch time of when the last time a sample was received for the specified device ID. If you find that
 * it's been a "long" time since a sample has been received from a device, you may check the device is powered and within range of
 * an access point, (only accurate while actively streaming data thru the host libraries).
 *
 * @param context
 * @param device_id The device ID (this is different then the Case ID on the back of the monitor)
 *
 * @return Zero on error of if no data has been received, non-zero with the epoch time otherwise.
 * @ingroup Context
 */
APDM_EXPORT time_t apdm_ctx_get_last_received_timestamp_for_device(apdm_ctx_t context, const uint32_t device_id);

/**
 * This function sets the maximum amount of delay allowable for data returned from the
 * host libraries. The default is 5ms. The max is 15 minutes.
 *
 * @param context The apdm handle to check data on
 * @param max_data_delay_seconds The maximum age of returned packets from the library, set APDM_INFINITE_MAX_LATENCY for infinity
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_set_max_sample_delay_seconds(apdm_ctx_t context, const uint16_t max_data_delay_seconds);

/**
 * This function sets the orientation model used for computing orientation estimates. 
 * The default is APDM_ORIENTATION_MODEL_ALL if all sensors are enabled, or 
 * APDM_ORIENTATION_MODEL_NO_MAG if the magnetometer is disabled.
 * 
 * @param context The apdm handle to set the orientation model on
 * @param orientation_model The orientation model to use: APDM_ORIENTATION_MODEL_ALL, 
     APDM_ORIENTATION_MODEL_UNDISTURBED_MAG, APDM_ORIENTATION_MODEL_NO_MAG
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_set_orientation_model(apdm_ctx_t context, const apdm_orientation_model_t orientation_model);

/**
 * Retrieves the latency of an individual monitor from the given context.
 *
 * @param context The apdm context
 * @param monitor_id The ID of the monitor for which you want to know the latency.
 * @param *dest The destination into which to store the latency value.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_monitor_latency(apdm_ctx_t context, const uint32_t monitor_id, int64_t *dest);





/**
 * Gets the current max-sample-delay setting, in seconds (aka max latency)
 *
 * @param context The apdm handle to check data on
 * @param *dest Destination into which the setting should be stored
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_max_sample_delay_seconds(apdm_ctx_t context, uint16_t *dest);

//FIXME document this
#if 0
APDM_EXPORT int apdm_ctx_set_sync_value(apdm_ctx_t context, const uint64_t new_sync_value);
#endif

/**
 * Used to reset the counter which tracks the number of samples received from the access point by the host libraries.
 * @param context The context
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_reset_num_samples_from_ap(apdm_ctx_t context);


/**
 * Returns the number of sensor samples that have been transfered from all the attached AP's to the host. This number
 * includes all samples that are currently in the correlation FIFO's of the library. It is useful if you want to verify
 * data transfer from device to access point to host, but where configured library processing policys might be causing delays
 * in data returned by the libraries.
 *
 * @param context The context
 *
 * @return If zero of positive, the number of samples that have been transfered from the AP to the host libraries since
 *         the last time aptm_reset_num_samples_from_ap was called, negative error code otherwise.
 * @ingroup Context
 */
APDM_EXPORT int apdm_ctx_get_num_samples_from_ap(apdm_ctx_t context);



//APDM Misc

/**
 * @return The version of the host libraries currently being used
 * @ingroup Misc
 */
APDM_EXPORT const char* apdm_get_library_version(void);

/**
 * @return The date on which the libraries were built.
 *
 * @ingroup Misc
 */
APDM_EXPORT const char* apdm_get_library_build_datetime(void);

/**
 * Returns a string representation of the version of libusb that is currently compiled and being used.
 *
 * @param *dest Destination string buffer into which to store the libusb version string
 * @param dest_buffer_length Length of the destination buffer.
 *
 * @return APDM_OK on success, error code otherwise.
 */
APDM_EXPORT int apdm_usb_get_libusb_version_string(char *dest, const uint32_t dest_buffer_length);

/**
 * Calculates the time delta in milliseconds between two sync values.
 *
 * @param sync_newer The larger of the two sync values
 * @param sync_older The smaller of the two sync values
 *
 * @return The number of milliseconds delta between the two passed synced value.
 * @ingroup DataHandling
 */
APDM_EXPORT uint64_t apdm_calculate_sync_value_age(const uint64_t sync_newer, const uint64_t sync_older);

/**
 * @return the number of milliseconds elapsed since the UNIX epoch. Works on both windows and linux.
 * @ingroup Misc
 */
APDM_EXPORT uint64_t apdm_get_time_ms_64(struct timeval *dest);

/**
 * Converts a sync value to an epoch second.
 *
 * @param sync_value The sync value
 *
 * @return The corresponding epoch second for the passed sync value.
 * @ingroup DataHandling
 */
APDM_EXPORT uint64_t apdm_epoch_access_point_to_epoch_second(const uint64_t sync_value);

/**
 * Converts a sync value to an epoch millisecond  (point in time, as
 * number of milliseconds since 1970, that the sample was taken).
 *
 * @param sync_value The sync value
 *
 * @return The corresponding epoch millisecond for the passed sync value (point in time, as
 * number of milliseconds since 1970, that the sample was taken).
 * @ingroup DataHandling
 */
APDM_EXPORT uint64_t apdm_epoch_access_point_to_epoch_millisecond(const uint64_t sync_value);

/**
 * Converts a sync value to an epoch second and microseconds (point in time, as
 * since 1970, that the sample was taken).
 *
 * @param sync_value The sync value
 * @param *dest Destination timeval struct into which to store the time,
 *
 * @return The corresponding epoch second and microseconds for the passed sync value (point in time, since 1970, that the sample was taken).
 * @ingroup DataHandling
 */
APDM_EXPORT int apdm_epoch_access_point_to_epoch_microsecond(const uint64_t sync_value, struct timeval *dest);

/**
 * Converts a sync value to an epoch second and microseconds (point in time, as
 * since 1970, that the sample was taken).
 *
 * @param sync_value The sync value
 *
 * @return The corresponding epoch second and microseconds for the passed sync value (point in time, since 1970, that the sample was taken).
 * @ingroup DataHandling
 */
APDM_EXPORT uint64_t apdm_epoch_access_point_to_epoch_microsecond2(const uint64_t sync_value);

/**
 * Helper function to convert an epoch second to a sync-value
 *
 * @param epochSecond Number of seconds since 1970, unix time.
 *
 * @return The system sync value that represents that point int time.
 * @ingroup DataHandling
 */
APDM_EXPORT uint64_t apdm_epoch_second_to_epoch_access_point(const uint64_t epoch_second);

uint64_t apdm_epoch_millisecond_to_epoch_access_point(const uint64_t epoch_millisecond);

/**
 * Helper function to convert an epoch micro-second to a sync-value
 *
 * @param epochMicroSecond Number of microseconds seconds since 1970, unix time.
 *
 * @return The system sync value that represents that point int time.
 * @ingroup DataHandling
 */
APDM_EXPORT uint64_t apdm_epoch_microsecond_to_epoch_access_point(const uint64_t epoch_micro_second);

/**
 * Helper function to convert an apdm status code to a string.
 *
 * @param status_code The status code for which you want the string representation.
 *
 * @return Pointer to a char array with a string representation of the status code.
 * @ingroup Misc
 */
APDM_EXPORT const char* apdm_strerror(const enum APDM_Status status_code);

/**
 * @param rate The apdm_monitor_output_select_rate_t for which you want the string representation
 *
 * @return The string representation of the rate passed in.
 * @ingroup Misc
 */
APDM_EXPORT const char* apdm_output_select_rate_t_str(const apdm_monitor_output_select_rate_t rate);

/**
 * @param rate The apdm_monitor_decimation_rate_t for which you want the string representation.
 *
 * @return The string representation of the rate passed in.
 * @ingroup Misc
 */
APDM_EXPORT const char* apdm_monitor_decimation_rate_t_str(const apdm_monitor_decimation_rate_t rate);

/**
 * @param rate The apdm_monitor_output_select_rate_t for which you want the numerical output rate.
 *
 * @return The output sample rate of the specified apdm_monitor_output_select_rate_t, e.g APDM_OUTPUT_SELECT_RATE_128 maps to 128
 * @ingroup Misc
 */
APDM_EXPORT uint32_t apdm_monitor_output_select_rate_t_to_int(const apdm_monitor_output_select_rate_t rate);

/**
 * @param rate For a given output rate, determine the expected sync delta between any two samples.
 *
 * @return The expected sync delta
 * @ingroup Misc
 */
APDM_EXPORT uint32_t apdm_monitor_get_expected_sync_delta(const apdm_monitor_output_select_rate_t rate);

/**
 * FIXME document this
 */
APDM_EXPORT uint32_t apdm_monitor_get_expected_sync_delta_raw(const uint32_t output_rate_hz);

#if 0
/** FIXME document this */
APDM_EXPORT apdm_monitor_output_select_rate_t apdm_monitor_int_to_output_select_rate_t(const uint32_t hz);
#endif

/**
 * @param rate The apdm_monitor_decimation_rate_t for which you want the numerical decimation rate.
 *
 * @return The decimation rate, numerical, for the specified rate, E.G. APDM_DECIMATE_5x2 maps to 10
 * @ingroup Misc
 */
APDM_EXPORT uint32_t apdm_monitor_decimation_rate_t_to_int(const apdm_monitor_decimation_rate_t rate);

/**
 * @param mode The apdm_wireless_mode_t for which you want the string representation of.
 * @return The string representation of the specified mode.
 * @ingroup Misc
 */
APDM_EXPORT const char* apdm_wireless_mode_t_str(const apdm_wireless_mode_t mode);


/**
 * Helper function to get the severity level of a given apdm status code (APDM_Status)
 *
 * @param status The APDM_Status status code in question
 *
 * @return APDM_SEVERITY_ERROR, APDM_SEVERITY_WARNING or APDM_SEVERITY_INFO depending on the respective error severity.
 * @ingroup Misc
 */
APDM_EXPORT enum APDM_Status_Severity apdm_error_severity(const int status);

/**
 * This function will estimate the current sync value of the system. This should be accurate to within about 50ms, and is dependant
 * on the timing latency of the USB bus on the host and the clock drift rate delta between the AP and the host computer.
 *
 * @param context The context
 *
 * @return An estimate of the current sync value, in units of 1/2560 seconds.
 * @ingroup Context
 */
APDM_EXPORT uint64_t apdm_ctx_estimate_now_sync_value(apdm_ctx_t context);


/**
 * This function will estimate the current V2 sync value of the system. This should be accurate to within about 50ms, and is dependant
 * on the timing latency of the USB bus on the host and the clock drift rate delta between the AP and the host computer.
 *
 * @param context The context
 *
 * @return An estimate of the current sync value (microseconds).
 * @ingroup Context
 */
APDM_EXPORT uint64_t apdm_ctx_estimate_now_sync_value_v2(apdm_ctx_t context);

	
	
/**
 * Recalibrates the magenteometers for bias shifts due to magnetization of monitor componenets.
 * @param file HDF5 file containing raw and calibrated data during a period of up to 5 minutes of rotation covering as much of the orientation space as possible in a uniform magnetic field
 * @param local_field_magnitude Local magnetic field strength, usually obtained through a geomagnetic model like ( http://www.ngdc.noaa.gov/geomagmodels/IGRFWMM.jsp ). Set to 0 to use the same value as last time the monitor was calibrated. 
 * @param calibration_block Byte array to be populated with the updated calibration data. Must be 2048 bytes. 
 * @param uncalibrated_data Output array containing the raw data used during recalibration calibrated with the original calibration data. 3*num_samples, column major ordering
 * @param calibrated_data Output array containing the raw data used during recalibration calibrated with the updated calibration data. 3*num_samples, column major ordering
 * @param num_samples Number of samples in the calibrated data arrays (acutal array is 3 times larger). Updated with the number of samples actually written to the arrays. Should be at least 38400 (5 minutes)
 * 
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataHandling
 */
APDM_EXPORT int apdm_recalibrate_magnetometers_from_h5(char *file, double local_field_magnitude, uint8_t *calibration_block, double *uncalibrated_data, double *calibrated_data, int32_t *num_samples);

/**
 * Recalibrates the gyroscopes for bias shifts.
 * @param file HDF5 file containing raw and calibrated data during a period of up to 5 minutes of sitting still on a table.
 * @param calibration_block Byte array to be populated with the updated calibration data. Must be 2048 bytes. 
 * 
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataHandling
 */
//APDM_EXPORT int apdm_recalibrate_magnetometers_from_h5(apdm_magnetometer_recalibration_t *mag_recalibration);
APDM_EXPORT int apdm_recalibrate_gyroscopes_from_h5(char *file, uint8_t *calibration_block);
	
/**
 * Reads metadata from a .apdm file and uses it to populate a apdm_recording_info_t structure.
 * @param filename The filename of the .apdm file
 * @param *recording_info A pointer to the file_info structure to populate with metadata.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */	
APDM_EXPORT int apdm_read_raw_file_info(const char *filename, apdm_recording_info_t *recording_info);

APDM_EXPORT int apdm_read_raw_file_info_num_samples(const char *file, uint32_t *dest);
APDM_EXPORT int apdm_read_raw_file_info_sample_rate(const char *file, uint32_t *dest);


/**
 * Reads metadata from a .apdm file and uses it to set the version parameter with a value from the apdm_file_version_t enum.
 * @param filename The filename of the .apdm file
 * @param *version A pointer to a variable to set with the version number of the file.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_get_apdm_file_version(const char * filepath, apdm_file_version_t *version);

/**
 * Reads a .apdm file, and writes either raw, calibrated, or both, to either a .csv or a .h5 file.
 * @param file_in An array of .apdm input file names from an unique monitors.
 * @param calibration_file array of optional files containing .hex calibration data for the monitors.
 *        If NULL, the calibration data in the .apdm file is used.
 * @param nFiles The number of input files present in file_in. 
 * @param file_out The output filename (should end in .csv or .h5). If NULL, .csv format is forced and output is written to stdout. 
 * @param store_raw If true, raw data is stored.
 * @param store_si If true, calibrated data (in SI units) is stored.
 * @param format_hdf If true, file_out will be written to in HDF5 format. If false, file_out will be written to in .csv format. If multiple input files are present, HDF output must be selected.
 * @param csv_delimiter Column delimiter to use if format_hdf is false (writing in csv format). 
 * @param progress If not null, this is an apdm_progress_t structure allocated by the caller and modified as this function runs.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_process_raw(char **file_in, char **calibration_file, int nFiles, const char *file_out,  const bool store_raw, const bool store_si, const bool format_hdf, const bool compress, char csv_delimiter, apdm_progress_t *progress);


/**
 * Reads a .apdm file, and writes either raw, calibrated, or both, to either a .csv or a .h5 file.
 * @param file_in An array of .apdm input file names from unique monitors.
 * @param calibration_file array of optional files containing .hex calibration data for the monitors.
 *        If NULL, the calibration data in the .apdm file is used.
 * @param nFiles The number of input files present in file_in. 
 * @param file_out The output filename (should end in .csv or .h5). If NULL, .csv format is forced and output is written to stdout. 
 * @param store_raw If true, raw data is stored.
 * @param store_si If true, calibrated data (in SI units) is stored.
 * @param store_filtered If true, filtered data (in SI units) is stored.
 * @param format_hdf If true, file_out will be written to in HDF5 format. If false, file_out will be written to in .csv format. If multiple input files are present, HDF output must be selected.
 * @param compress Compress data in HDF5 file output.
 * @param csv_delimiter Column delimiter to use if format_hdf is false (writing in csv format). 
 * @param progress If not null, this is an apdm_progress_t structure allocated by the caller and modified as this function runs.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_process_raw2(char **file_in, char **calibration_file, int nFiles, const char *file_out,  const bool store_raw, const bool store_si, const bool store_filtered, const bool format_hdf, const bool compress, char csv_delimiter, apdm_progress_t *progress);
    
/**
 * Reads a .apdm file, and writes either raw, calibrated, or both, to either a .csv or a .h5 file.
 * @param params An apdm_file_conversion_parameter_t struct containing parameters for file conversion. Should be initialized with apdm_initialize_file_conversion_parameters().
 *
 * @return APDM_OK on success, error code otherwise
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_process_raw3(apdm_file_conversion_parameter_t *params);

/**
 * Reads a .h5 file, and converts it to a .csv file.
 * @param h5file The path to the h5 file to convert
 * @param csvfile The path to the csv file to create
 * @param delimiter Delimiter to use between columns in the csv file
 *
 * @return APDM_OK on success, error code otherwise
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_convert_h5_to_csv(char *h5file, char *csvfile, char delimiter);

    
    
/**
 * Releases memory stored in the file_in_struct conversion parameter.
 * @param params An apdm_file_conversion_parameter_t struct containing parameters for file conversion.
 *
 * @return APDM_OK on success, error code otherwise
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_release_conversion_parameters(apdm_file_conversion_parameter_t *params);

/**
 * Finds the first and last common sample within the raw apdm files passed in.
 * @param files_in An array of .apdm input file names from unique monitors.
 * @param nFiles The number of input files present in file_in. 
 * @param first_sample Output parameter indicating the first common sample. Default = 1
 * @param last_sample Output parameter indicating the last common sample. Default = 1e15
 * @param use_sync_lock Input parameter indicating whether sync_lock bit should be used to determine start and stop times
 *
 * @return APDM_OK on success, error code otherwise
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_find_first_and_last_common_samples(char **files_in, const int nFiles, uint64_t *first_sample, uint64_t *last_sample, const int use_sync_lock);

/**
 * Iteratively finds button transitions (0 to 1) within a raw .apdm input file.
 * @param file A string indicating the full file path to a .apdm file.
 * @param recording_info The recording_info structure for the file. This is populated by first calling the apdm_read_file_info() method. 
 * @param time_of_button_transition An In-Out parameter indicating the start sync_val to look for (In) and the sync_val of the found transition. If no transition is found, 0 is returned. Should be '1' the first iteration of the function call.
 * @param start_position An In-Out parameter indicating the file position to start searching in (In) and where the last search terminated (out)
 * @param mode 1: Look for button presses, 0: Look for button releases (v1 only).
 * @param BYTE_ARRAY string from the recording_info structure representing the button label. (v2 only)
 * @param description_size size of the BYTE_ARRAY buffer. (v2 only)
 *
 * @return APDM_OK on success, error code otherwise
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_find_button_transition(const char *file, apdm_recording_info_t *recording_info, uint64_t *time_of_button_transition, int64_t *start_position, uint8_t mode, char *BYTE_ARRAY, int description_size);

/**
 * Initializes an apdm_file_conversion_paraemter_t structure with default values. Further modifications can be made to the parameters
 * @param params The structure to initialize 
 *
 * @return APDM_OK on success, error code otherwise
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_initialize_file_conversion_parameters(apdm_file_conversion_parameter_t *params);
    
/**
 * Creates a new APDM HDF5 file and opens it for writing. Used with apdm_write_record_hdf, apdm_write_annotation, and apdm_close_file_hdf to stream to a data file.
 * @param filename The filename to create. Should have the ".h5" extension. 
 * @param device_info The configuration information for each monitor. Used to write various metadata. This should be a pointer to an array of apdm_device_info_t structures.
 * @param nMonitors The number of monitors in the device_info array.
 *
 * @return hid_t HDF5 file handle, always greater than 0 on success.
 * @ingroup DataFiles
 */
APDM_EXPORT hid_t apdm_create_file_hdf(const char *filename, apdm_device_info_t *device_info, const int nMonitors);
    
/**
 * Creates a new APDM HDF5 file and opens it for writing. Used with apdm_write_record_hdf, apdm_write_annotation, and apdm_close_file_hdf to stream to a data file.
 * @param filename The filename to create. Should have the ".h5" extension.
 * @param context The active context, which provides information about the currently configured monitors.
 *
 * @return hid_t HDF5 file handle, always > 0 on success.
 * @ingroup DataFiles
 */
APDM_EXPORT hid_t apdm_ctx_create_file_hdf(const char *filename, apdm_ctx_t context);

    
/**
 * Creates a new APDM HDF5 file and opens it for writing. Used with apdm_write_record_hdf, apdm_write_annotation, and apdm_close_file_hdf to stream to a data file.
 * @param filename The filename to create. Should have the ".h5" extension.
 * @param device_info The configuration information for each monitor. Used to write various metadata. This should be a pointer to an array of apdm_device_info_t structures.
 * @param nMonitors The number of monitors in the device_info array.
 * @param compress_data Flag to indicate whether compression filters should be applied to the datasets
 *
 * @return hid_t HDF5 file handle, always greater than 0 on success.
 * @ingroup DataFiles
 */
APDM_EXPORT hid_t apdm_create_file_hdf2(const char *filename, apdm_device_info_t *device_info, const int nMonitors, const bool compress_data);

/**
 * Creates a new APDM HDF5 file and opens it for writing. Used with apdm_write_record_hdf, apdm_write_annotation, and apdm_close_file_hdf to stream to a data file.
 * @param filename The filename to create. Should have the ".h5" extension.
 * @param device_info The configuration information for each monitor. Used to write various metadata. This should be a pointer to an array of apdm_device_info_t structures.
 * @param nMonitors The number of monitors in the device_info array.
 * @param compress_data Flag to indicate whether compression filters should be applied to the datasets
 * @param store_all_sensors Flag to incidate whether individual accelerometers (low and high) should be stored in addition to fused acceleration
 *
 * @return hid_t HDF5 file handle, always greater than 0 on success.
 * @ingroup DataFiles
 */
APDM_EXPORT hid_t apdm_create_file_hdf3(const char *filename, apdm_device_info_t *device_info, const int nMonitors, const bool compress_data, const bool store_all_sensors);


/**
 * Closes a file previously opened with apdm_create_file_hdf.
 * @param file The HDF5 file handle returned from apdm_create_file_hd
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_close_file_hdf(hid_t file);

/**
 * Creates a new file and returns a file pointer.
 * @param filename Name of the file to create
 *
 * @return apdm_csv_t (actually a FILE *) file handle for the new file
 * @ingroup DataFiles
 */
APDM_EXPORT apdm_csv_t apdm_create_file_csv(char *filename);

/**
 * Close a file opened with apdm_create_file_csv.
 * @param file_handle The file handle returned from apdm_create_file_csv
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_close_file_csv(apdm_csv_t file_handle);


/**
 * Write an ranging measurement to a HDF5 file (previously created with apdm_create_file_hdf).
 *
 * @param file The HDF5 file handle returned from apdm_create_file_hdf
 * @param sensor The name of the sensor group in the HDF5 file (e.g. "XI-000123")
 * @param anchor The anchor ID (e.g. 15)
 * @param epoch_time The epoch time in microseconds at which the measurement was taken
 * @param anchor_timestamp The decawave timestamp on the anchor when the ranging packet was recieved
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
int apdm_write_ranging_sample_hdf(hid_t file_handle, char *sensor, char *anchor, uint64_t epoch_time, uint64_t anchor_timestamp);

/**
 * Write an array of records to a HDF5 file (previously created with apdm_create_file_hdf). The sample number 
 * must be tracked by the caller, and incremented for each new sample. In case of dropped data, it may be 
 * desired to increment the sampleNumber for each dropped sample. It is assumed that there are a total of
 * nDevices samples, with one sample from each device.
 *
 * @param file The HDF5 file handle returned from apdm_create_file_hdf
 * @param info Array of apdm_device_info_t structs containing information about each monitor.
 * @param records Array of apdm_record_t structs containing the data for one sample set of each monitor.
 * @param sampleNumber An index into the arrays stored in the HDF5 file. It is important that the first time this function is called sampleNumber is 0.
 * @param nDevices Number of monitors in the info and records arrays.
 * @param store_raw Flag indicating whether raw data should be stored. (True: yes, False: no)
 * @param store_si Flag indicating whether SI data should be stored. (True: yes, False: no)
 * @param compress Flag indicating whether data should be compressed. This is almost always a good idea, but some old versions of Matlab (less than 2008b) have been found to have difficulty reading compressed data. (True: yes, False: no)
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_write_record_hdf(hid_t file, apdm_device_info_t *info, apdm_record_t *records, int sampleNumber, int nDevices, bool store_raw, bool store_si, bool compress);

/**
 * Write an array of records to a HDF5 file (previously created with apdm_create_file_hdf). The sample number 
 * must be tracked by the caller, and incremented for each new sample. In case of dropped data, it may be 
 * desired to increment the sampleNumber for each dropped sample.
 *
 * @param file The HDF5 file handle returned from apdm_create_file_hdf
 * @param info Array of apdm_device_info_t structs containing information about each monitor.
 * @param records Array of apdm_record_t structs containing the data for one sample set of each monitor.
 * @param sampleNumber An index into the arrays stored in the HDF5 file. It is important that the first time this function is called sampleNumber is 0.
 * @param nDevices Number of monitors in the info and records arrays.
 * @param store_raw Flag indicating whether raw data should be stored. (True: yes, False: no)
 * @param store_si Flag indicating whether SI data should be stored. (True: yes, False: no)
 * @param store_filtered Flag indicating whether filtered data should be stored. (True: yes, False: no)
 * @param compress Flag indicating whether data should be compressed. This is almost always a good idea, but some old versions of Matlab (less than 2008b) have been found to have difficulty reading compressed data. (True: yes, False: no)
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_write_record_hdf2(hid_t file, apdm_device_info_t *info, apdm_record_t *records, int sampleNumber, int nDevices, bool store_raw, bool store_si, bool store_filtered, bool compress);

/**
 * Write an array of records to a HDF5 file (previously created with apdm_create_file_hdf). The sample number 
 * must be tracked by the caller, and incremented for each new sample. In case of dropped data, it may be 
 * desired to increment the sampleNumber for each dropped sample. It is assumed that there are a total of
 * nDevices * nSamples total records, where the records are grouped by device. For example, if there are two
 * records from each of three devices, they would be ordered as: [D1S1, D1S2, D13, D2S1, D2S2, D2S3]
 *
 * @param file The HDF5 file handle returned from apdm_create_file_hdf
 * @param info Array of apdm_device_info_t structs containing information about each monitor.
 * @param records Array of apdm_record_t structs containing the data for one sample set of each monitor.
 * @param sampleNumber An index into the arrays stored in the HDF5 file. It is important that the first time this function is called sampleNumber is 0.
 * @param nDevices Number of monitors in the info and records arrays.
 * @param store_raw Flag indicating whether raw data should be stored. (True: yes, False: no)
 * @param store_si Flag indicating whether SI data should be stored. (True: yes, False: no)
 * @param store_filtered Flag indicating whether filtered data should be stored. (True: yes, False: no)
 * @param compress Flag indicating whether data should be compressed. This is almost always a good idea, but some old versions of Matlab (less than 2008b) have been found to have difficulty reading compressed data. (True: yes, False: no)
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_write_record_list_hdf(hid_t file_handle, apdm_device_info_t *info, apdm_record_t *records, const int sampleNumber, const int nDevices, const int nSamples, const bool store_raw, const bool store_si, const bool compress);
    
/**
 * Write the most recent record list to a HDF5 file (previously created with apdm_create_file_hdf).
 * It is assumed that apdm_ctx_get_next_access_point_record_list() has already been called and that
 * data exists to be read. The sample number must be tracked by the caller, and incremented for each
 * new sample. In case of dropped data, it may be desired to increment the sampleNumber for each dropped
 * sample.
 * @param file The HDF5 file handle returned from apdm_create_file_hdf
 * @param context The active context.
 * @param records Array of apdm_record_t structs containing the data for one sample set of each monitor.
 * @param sampleNumber An index into the arrays stored in the HDF5 file. It is important that the first time this function is called sampleNumber is 0.
 * @param store_raw Flag indicating whether raw data should be stored. (True: yes, False: no)
 * @param store_si Flag indicating whether SI data should be stored. (True: yes, False: no)
 * @param store_filtered Flag indicating whether filtered data should be stored. (True: yes, False: no)
 * @param compress Flag indicating whether data should be compressed. This is almost always a good idea, but some old versions of Matlab (<2008b) have been found to have difficulty reading compressed data. (True: yes, False: no)
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_ctx_write_record_hdf(hid_t file_handle, apdm_ctx_t context, const int sampleNumber, const bool store_raw, const bool store_si, const bool store_filtered, const bool compress);

/**
 * Write an array of records to a CSV file (previously created with apdm_create_file_csv). The sample number 
 * must be tracked by the caller, and incremented for each new sample.
 *
 * @param file The file handle returned from apdm_create_file_csv
 * @param info Array of apdm_device_info_t structs containing information about each monitor.
 * @param records Array of apdm_record_t structs containing the data for one sample set of each monitor.
 * @param sampleNumber It is important that the first time this function is called sampleNumber is 0. This can not be used as a row index. One row is created every time this function is called, reguardless of the value of sampleNumber. The meta data written in the first column depends on sampleNumber.
 * @param nDevices Number of monitors in the info and records arrays.
 * @param store_raw Flag indicating whether raw data should be stored. (True: yes, False: no)
 * @param store_si Flag indicating whether SI data should be stored. (True: yes, False: no)
 * @param delimiter Delimiter to use to separate columns. 
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_write_record_csv(apdm_csv_t file, apdm_device_info_t *info, apdm_record_t *records, int sampleNumber, int nDevices, bool store_raw, bool store_si, char delimiter);

/**
 * Adds an annotation to the HDF5 file.
 *
 * @param file The HDF5 file handle returned by apdm_create_file_hdf
 * @param annotation An apdm_annotation_t struct containing a monitor ID, timestamp (epoch microseconds), and string (max 2048 characters).
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_write_annotation(hid_t file, apdm_annotation_t *annotation);
	
/**
 * Helper function for working with HDF5 files. Loads a segment of data from one of the datasets in one of the monitors stored in the .h5 file.
 * @param file The .h5 file to load data from
 * @param monitor_id The group name for the monitor (returned by apdm_get_hdf_device_list). For v1 files, this is the 'Opal_xx' where xx is the monitor id. For v2 files, this is the case id.
 * @param datasetName The name of the dataset to load. Must be ("Accelerometers", "Gyroscopes", "Magnetometers", or "Temperature");
 * @param data Output array to load data into. This array is "flattened" so that the nth sample of the mth channel is at location data[n+m*N] where N is the total number of samples for each channel in the array.
 * @param ndims Number of dimensions in the dataset to be read. Should always be 2.
 * @param start_index Initial index to start reading from. First element is sample number, second element is channel number.
 * @param shape Size of the output array (data). First element is the number of samples, second is the number of channels.
 * @param strideLength Number of data points to skip by in each dimension.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT	int apdm_read_hdf_dataset(const char *file, char *monitor_id,const char *datasetName, double *data, int ndims, const int *start_index, const int *shape, const int *strideLength);

/**
 * Helper function for working with HDF5 files. Loads a segment of data from one of the time datasets in one of the monitors stored in the .h5 file.
 * @param file The .h5 file to load data from
 * @param monitor_id The group name for the monitor (returned by apdm_get_hdf_device_list). For v1 files, this is the 'Opal_xx' where xx is the monitor id. For v2 files, this is the case id.
 * @param datasetName The name of the dataset to load. Must be "Time" or "SyncValue"
 * @param data Output array to load data into. 
 * @param start_index Initial index to start reading from
 * @param nSamples Size of the output array (data). 
 * @param strideLength Number of data points to skip by.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */	
APDM_EXPORT	int apdm_read_hdf_timestamps(char *file, char *monitor_id, char *datasetName, uint64_t *data, int start_index, int nSamples, int strideLength);	
	
/**
 * Helper function for working with HDF5 files. Gets the size of a specified dataset.
 *
 * @param file The .h5 file to inspect.
 * @param monitor_id The group name for the monitor (returned by apdm_get_hdf_device_list). For v1 files, this is the 'Opal_xx' where xx is the monitor id. For v2 files, this is the case id.
 * @param datasetName The name of the dataset to inspect. Must be "Accelerometers", "Gyroscopes", "Magnetometers", "Temperature", )
 * @param shape Output array containing the size in each dimension of the datset. If shape is NULL, then only ndims will be set. 
 * @param ndims Output containing the number of dimensions in the dataset.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT	int apdm_get_hdf_dataset_shape(char *file, char *monitor_id, char *datasetName, int *shape, int *ndims);

/**
 * Helper function for working with HDF5 files. Gets the calibration data from a file and converts it to an apdm_sensor_compensation_t struct.
 * 
 * @param file The .h5 file to read.
 * @param case_id The Case ID string of the monitor to get the calibration data for.
 * @param sensor_comp apdm_sensor_compensation_t structure to be populated with the calibration data.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT int apdm_read_hdf_calibration_data(char *file, char *case_id, apdm_sensor_compensation_t *sensor_comp);
	
	
/**
 * Helper function for working with HDF5 files. Gets the list of monitor IDs stored in the file.
 * 
 * @param file The .h5 file to inspect.
 * @param **monitor_ids Output array containing the group name for each monitor in the file. If NULL, only nDevices will be set.
 * @param *nDevices Output number of monitors in the file.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT	int apdm_get_hdf_device_list(char *file, char **monitor_ids, int *nDevices);

/**
 * Helper function for working with HDF5 files that is compatible with SWIG based Java bindings. Gets the list of monitor IDs stored in the file.
 * 
 * @param file The .h5 file to inspect.
 * @param *monitor_ids Output array containing the group name for each monitor in the file. If NULL, only nDevices will be set.
 * @param *nDevices Output number of monitors in the file.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT	int apdm_get_hdf_device_list_swig(char *file, apdm_case_id_t *monitor_ids, int *nDevices);

/**
 * Helper function for working with HDF5 files. Gets the list of monitor labels stored in the file. This list is in the same order as the list returned by apdm_get_hdf_device_list().
 * 
 * @param file The .h5 file to inspect.
 * @param monitor_labels Output array containing the user specified label for each monitor in the file. If NULL, only nDevices will be set.
 * @param nDevices Output number of monitors in the file.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT	int apdm_get_hdf_label_list(char *file, char **monitor_labels, int *nDevices);

/**
 * Helper function for working with HDF5 files that is compatible with SWIG based Java bindings. Gets the list of monitor labels stored in the file. This list is in the same order as the list returned by apdm_get_hdf_device_list().
 * 
 * @param file The .h5 file to inspect.
 * @param monitor_labels Output array containing the user specified label for each monitor in the file. If NULL, only nDevices will be set.
 * @param nDevices Output number of monitors in the file.
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup DataFiles
 */
APDM_EXPORT	int apdm_get_hdf_label_list_swig(char *file, apdm_monitor_label_t *monitor_labels, int *nDevices);
	
///**
// * Helper function for working with HDF5 files. Reads all of the annotations stored in the .h5 file.
// * @param file The .h5 file to load data from
// * @param annotations Array of apdm_annotation_t structures containing the annotations. If NULL, only nAnnotations is set. 
// * @param nAnnotations Number of annotations in the file. 
// * @return APDM_OK on success
// * @ingroup APDM
// */	
//APDM_EXPORT	int apdm_read_hdf_annotations(char *file, apdm_annotation_t *annotations, int *nAnnotations);
	

/**
 * Platform independent version of usleep().
 *
 * @param microseconds Number of microseconds to sleep for
 * @ingroup Misc
 */
APDM_EXPORT void apdm_usleep(const uint64_t microseconds);

/**
 * Platform independent version of msleep().
 *
 * @param milliseconds Number of milliseconds to sleep for
 * @ingroup Misc
 */
APDM_EXPORT void apdm_msleep(const uint64_t milliseconds);




//Logging
/**
 * Sets the current log level to be used by APDM libraries. Valid values are:
 *
 * @param log_level Valid values are:
 *  APDM_LL_ALL = 0,
 *  APDM_LL_DEBUG = 1,
 *  APDM_LL_INFO = 2,
 *  APDM_LL_WARNING = 3,
 *  APDM_LL_ERROR = 4,
 *  APDM_LL_NONE = 5
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_set_log_level(int log_level);



/**
 * @param level The level for which you want the string representation
 *
 * @return Pointer to string for the given log level
 *
 * @ingroup Logging
 */
APDM_EXPORT const char* apdm_logging_level_t_str(const apdm_logging_level_t level);

/**
 * Sets and opens a log file to be used by APDM libraries for logging purposes
 *
 * @param filePath The file to which logging data should be saved
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_set_log_file(const char *filePath);

/**
 * Closes the apdm log file (if its open)
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_close_log_file(void);

/**
 * Adds log message to the apdm log stream at DEBUG level.
 *
 * @param format printf-style format string
 * @param ... var-args for printf-style values
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_log(const char *format, ...);


/**
 * Adds a log message to the adpm loc stream at the given loglevel using the format and args passed in.
 *
 * @param level The log level that the message should be logged at.
 * @param format printf-style format string
 * @param ... var-args for printf-style values
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_logl(const enum APDM_Logging_Level level, const char *format, ...);


/**
 * Adds log message to the apdm log stream at DEBUG level.
 *
 * @param format printf-style format string
 * @param ... var-args for printf-style values
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_log_debug(const char *format, ...);

/**
 * Adds log message to the apdm log stream at INFO level.
 *
 * @param format printf-style format string
 * @param ... var-args for printf-style values
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_log_info(const char *format, ...);

/**
 * Adds log message to the apdm log stream at WARNING level.
 *
 * @param format printf-style format string
 * @param ... var-args for printf-style values
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_log_warning(const char *format, ...);

/**
 * Adds log message to the apdm log stream at ERROR level.
 *
 * @param format printf-style format string
 * @param ... var-args for printf-style values
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_log_error(const char *format, ...);

/**
 *
 */
APDM_EXPORT int apdm_log_device_info(const uint32_t device_id, apdm_device_info_t *info_ptr, const enum APDM_Logging_Level level);

/**
 * Logs all the details about the passed context to the apdm logging stream
 *
 * @param context The context to be logged
 * @param level The logging severity level to log as
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 * @ingroup Logging
 */
APDM_EXPORT int apdm_log_context(apdm_ctx_t context, const enum APDM_Logging_Level level);


/**
 * Recomputes orientation estimates for file_in
 */
APDM_EXPORT int apdm_reprocess_orientation(char *file_in);
APDM_EXPORT int apdm_reprocess_orientation2(char *file_in, apdm_orientation_model_t orientation_model);
APDM_EXPORT int apdm_reprocess_orientation3(char *file_in, apdm_orientation_model_t orientation_model, bool use_v1_algorithm);
    
/** Reads the data in calibration_data to the specified device
  * @param device_handle The handle to the device to read the calibration data from
  * @param src A pointer to the calibration structure to update with the device's calibration data
  * @param user_cal_data_flag If true, the user calibration data is read. If false, the device's default calibration is read.
  */
APDM_EXPORT int apdm_read_calibration_data(apdm_device_handle_t device_handle, calibration_data_t *dest, const bool user_cal_data_flag);

/**
  * Writes the data in calibration_data to the specified device
  * @param device_handle The handle to the device to write the calibration data to
  * @param src A pointer to the calibration structure to write to the device's calibration data
  * @param calibration_type Type of cal data to be written to the V2 device.
  */
APDM_EXPORT int apdm_write_calibration_data_v2(apdm_device_handle_t h, const calibration_data_t *src, const calibration_type_t calibratin_type);

/**
  * Writes the data in calibration_data to the specified device
  * @param device_handle The handle to the device to write the calibration data to
  * @param src A pointer to the calibration structure to write to the device's calibration data
  * @param user_cal_data_flag If true, the user calibration data is written. If false, the device's default calibration is written
  * @deprecated Use apdm_write_calibration_data_v2() instead.
  */
APDM_DEPRECATED APDM_EXPORT int apdm_write_calibration_data(apdm_device_handle_t device_handle, const calibration_data_t *src, const bool user_cal_data_flag);


/**
 * Write data integrity status information to an HDF file. The integrity status is stored on an apdm_record_t structure.
 *
 * @param file_handle The HDF5 file handle returned from apdm_create_file_hdf
 * @param case_id The numeric case ID of the Opal where the issue originated
 * @param status A 32-bit flag indicating which, if any, integrity check statuses were detected for the sample/Opal.
 * @param sync_val The sync val for the sample whose integrity check is being logged
 *
 * @return APDM_OK on success, error code from 'enum APDM_Status' in apdm_types.h
 */
APDM_EXPORT int apdm_hdf_write_data_status(const hid_t file_handle, const char *case_id, const uint32_t status, const uint64_t sync_value);




#ifdef __cplusplus
}
#endif

#endif /*APDM_H_*/
