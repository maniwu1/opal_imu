#ifndef APDM_TYPES_H_
#define APDM_TYPES_H_

#include "apdm_stdincludes.h"
#include "v2_common.h"
#include "H5Apublic.h"

#ifdef __cplusplus
extern "C" {
#endif


#define APDM_WIRELESS_CHANNEL_SPACING      7
#define APDM_SYNC_COUNTS_PER_SECOND        2560

#ifndef MAX
#undef MAX
#define MAX(a,b)        ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#undef MIN
#define MIN(a,b)        ((a) < (b) ? (a) : (b))
#endif

#define APDM_DEFAULT_MAX_LATENCY            0xFFFF
#define APDM_INFINITE_MAX_LATENCY           0xFFFF
#define APDM_DEFAULT_MAX_LATENCY_SECONDS    0xFFFF 

#define APDM_MAX_WIRELESS_CHANNEL           95


#ifndef _STDINT_H
//Note: These typedefs are here to allow Matlab to map the standard items in stdint.h to c-language primative data types, for some reason matlab is not pulling in the stdint.h library, FIXME, investigate that
#define uint8_t   unsigned char
#define uint16_t   unsigned short
#define uint32_t   unsigned int 
#define int32_t   int 
#define int16_t   short
#define int8_t   char
#define uint64_t unsigned long long int
#define int64_t long long int
#endif

#define DEVICE_LABEL_SIZE                16
#define CASE_ID_SIZE					 16
#define VERSION_STRING_SIZE              256
#define TIMEZONE_STRING_SIZE             40
#define CALIBRATION_DATA_BUFFER_SIZE     2048
#define MAX_APDM_EVENTS                  20
#define ADPM_DEVICE_COMMUNICATIONS_HANDLE_NEW_T_INITIALIZER    NULL


#define APDM_MAX_NUMBER_OF_SENSORS                               36

#define MAX_SAMPLES_THAT_A_DEVICE_CAN_BUFFER_WO_SD_CARD          30

//#define MAX_SAMPLES_THAT_A_DEVICE_CAN_BUFFER                     57600

//128 hz * (seconds) * (minutes) * (hours of battery life)
#define MAX_SAMPLES_THAT_A_DEVICE_CAN_BUFFER                     (128 * 60 * 60 * 14)

// Masks for monitor data flags field
#define MONITOR_DATA_FLAG_ACCEL 	  	    0x0001
#define MONITOR_DATA_FLAG_GYRO 	 	        0x0002
#define MONITOR_DATA_FLAG_MAG 	 	        0x0004
#define MONITOR_DATA_FLAG_FULL_SCALE 	    0x0008
#define MONITOR_DATA_FLAG_TEMP_SELECT       0x0010
#define MONITOR_DATA_FLAG_BUTTON_STATE 	 	0x0020
#define MONITOR_DATA_FLAG_SYNC_LOCK 	 	0x0040
#define MONITOR_DATA_FLAG_SYNC_RESET 	 	0x0080
#define MONITOR_DATA_FLAG_OPT_SELECT_0 	0x1000
#define MONITOR_DATA_FLAG_OPT_SELECT_1 	0x2000
#define MONITOR_DATA_FLAG_OPT_SELECT_2 	0x4000
#define MONITOR_DATA_FLAG_OPT_SELECT_3 	0x8000


//====================================================================
// Motion Monitor Enums
//====================================================================


/**
 * These are error codes originating from the motion monitor.
 */
typedef enum {
	E_OK = 0,
	E_EVENT_OVERFLOW = 1,
	E_BUFFER_OVERFLOW = 2,
	E_SD_RESPONSE_TIMEOUT = 3,
	E_SD_R1_INVALID = 4,
	E_SD_R1_PARAM_ERROR = 5,
	E_SD_R1_ADDRESS_ERROR = 6,
	E_SD_R1_ERASE_SEQ_ERROR = 7,
	E_SD_R1_CRC_ERROR = 8,
	E_SD_R1_ILLEGAL_CMD = 9,
	E_SD_R1_ERASE_RESET = 10,
	E_SD_CMD_0 = 11,
	E_SD_CMD_8 = 12,
	E_SD_CMD_55 = 13,
	E_SD_CMD_41 = 14,
	E_SD_CMD_16 = 15,
	E_SD_CMD_24 = 16,
	E_SD_CMD_17 = 17,
	E_SD_CMD_9 = 18,
	E_SD_CMD_OTHER = 19,
	E_SD_BAD_SIZE = 20,
	E_SD_STATE = 21,
	E_SD_BUFFER_FULL = 22,
	E_SD_CMD_8_FORMAT = 23,
	E_SD_CMD_8_VOLTAGE = 24,
	E_SD_CMD_8_CHECK = 25,
	E_SD_WRITE_CRC = 26,
	E_SD_READ_CRC = 27,
	E_SD_CSD_CRC = 28,
	E_SD_WRITE_ERROR = 29,
	E_SD_WRITE_OTHER = 30,
	E_SD_PROCESS = 31,
	E_UART_COLLISION = 32,
	E_DECIMATE_OVERFLOW1 = 33,
	E_DECIMATE_OVERFLOW2 = 34,
	E_DECIMATE_NEG1 = 35,
	E_DECIMATE_NEG2 = 36,
	E_WIRELESS_BUFFER_OVERFLOW = 37,
	E_SPI_TIMEOUT = 38,
	E_DEBUG_1 = 39,
	E_DEBUG_2 = 40,
	E_DEBUG_3 = 41,
	E_DEBUG_4 = 42,
	E_DEBUG_5 = 43,
	E_DEBUG_6 = 44,
	E_DEBUG_7 = 45,
	E_DEBUG_8 = 46,
	E_DEBUG_9 = 47,
	E_DEBUG_10 = 48,
	E_DEBUG_11 = 49,
	E_DEBUG_12 = 50,
	E_DEBUG_13 = 51,
	E_DEBUG_14 = 52,
	E_DEBUG_15 = 53,
	E_DEBUG_16 = 54,
	E_OSC_FAULT = 55,
	E_DCO_FAULT = 56,
	E_LF_FAULT = 57,
	E_XT1_FAULT = 58,
	E_OFF_BATTERY = 59,
	E_OFF_SPIN = 60,
	E_OFF_ALWAYS = 61,
	E_OFF_WIRELESS = 62,
	E_OFF_HALT = 63,
	E_WDT_RESET = 64,
	E_INVALID_CONFIG = 65,
	E_DEBUG_17 = 66,
	E_DEBUG_18 = 67,
	E_DEBUG_19 = 68,
	E_DEBUG_20 = 69,
	E_DEBUG_21 = 70,
	E_DEBUG_22 = 71,
	E_DEBUG_23 = 72,
	E_DEBUG_24 = 73,
	E_DEBUG_25 = 74,
	E_DEBUG_26 = 75,
	E_DEBUG_27 = 76,
	E_DEBUG_28 = 77,
	E_DEBUG_29 = 78,
	E_DEBUG_30 = 79,
	E_DEBUG_31 = 80,
	E_DEBUG_32 = 81,
	E_ISR_OVER_RUN = 82,
	E_STANDBY = 83
} apdm_monitor_error_id_t;

typedef enum {
	WIRELESS_PROTO_SYNC = 0,
	WIRELESS_PROTO_STREAM,
	WIRELESS_PROTO_SPEW,
	WIRELESS_PROTO_CARRIER,
	WIRELESS_PROTO_CARRIER_SWEEP,
	WIRELESS_PROTO_STREAM_MUX,
	WIRELESS_PROTO_SUPER_SPEW,
	WIRELESS_PROTO_CONFIG,
	WIRELESS_PROTO_CLOCK_CAL,
	WIRELESS_PROTO_ACCESS_POINT
} apdm_wireless_mode_t;

typedef enum {
	COM_DOCK_STATUS_WAIT = 0,
	COM_DOCK_STATUS_READY,
	COM_DOCK_STATUS_ERROR
} apdm_monitor_dock_status_t;

typedef enum {
	MONITOR_OPAL = 0,
	MONITOR_EMERALD,
	MONITOR_SAPHIRE
} apdm_monitor_type_t;

typedef enum {
	OFF_REASON_NONE = 0, OFF_REASON_BATTERY, OFF_REASON_SPIN, OFF_REASON_HALT, OFF_REASON_WIRELESS, OFF_REASON_ALWAYS, OFF_REASON_STANDBY
} apdm_monitor_off_reason_t;

/**This is used when setting and configuring various options on an monitor*/
enum APDMDeviceConfig {
	CONFIG_ENABLE_ACCEL = 0,
	CONFIG_ENABLE_GYRO = 1,
	CONFIG_ENABLE_MAG = 2,
	CONFIG_ENABLE_WIRELESS = 3,
	CONFIG_ENABLE_SD = 4,
	CONFIG_WIRELESS_CHANNEL_0 = 5,
	CONFIG_WIRELESS_CHANNEL_1 = 6,
	CONFIG_WIRELESS_CHANNEL_2 = 7,
	CONFIG_WIRELESS_CHANNEL_3 = 8,
	CONFIG_WIRELESS_ADDR_ID = 9,
	CONFIG_WIRELESS_ADDR_BLOCK_0 = 10,
	CONFIG_WIRELESS_ADDR_BLOCK_1 = 11,
	CONFIG_WIRELESS_ADDR_BLOCK_2 = 12,
	CONFIG_WIRELESS_ADDR_BLOCK_3 = 13,
	CONFIG_WIRELESS_TIME_SLICE = 14,
	CONFIG_WIRELESS_PROTOCOL = 15, /**< Values are from enum apdm_monitor_wireless_mode_t*/
	CONFIG_ACCEL_FULL_SCALE = 16,
	CONFIG_ALWAYS_OFF = 17,
	CONFIG_SPIN_MODE = 18,
	CONFIG_BATTERY_LED = 19,
	CONFIG_WIRELESS_LATENCY = 20,
	CONFIG_TEMP_SELECT = 21,
	CONFIG_OUTPUT_SELECT = 22, /**<Values are from enum apdm_monitor_output_select_rate_t*/
	CONFIG_DECIMATION_SELECT = 23, /**<Values are from enum apdm_monitor_decimation_rate_t*/
	CONFIG_BYPASS_DECIMATION = 24,
	CONFIG_EXTEND_LED = 25,
	CONFIG_MAG_SET_RESET = 26,
	CONFIG_LOCAL_TIMEZONE = 27,
  CONFIG_ENABLE_LED = 28,
  CONFIG_ENABLE_BUTTON = 29,
  CONFIG_BUTTON_MODE = 30,
  CONFIG_LABEL_0 = 31,
  CONFIG_LABEL_1 = 32,
  CONFIG_LABEL_2 = 33,
  CONFIG_LABEL_3 = 34,
  CONFIG_DEBUG_LED = 35,
  CONFIG_DEBUG_FILE = 36,
  CONFIG_BATTERY_CUTOFF = 37
};

typedef enum APDM_Decimation_Rates {
	APDM_DECIMATE_1x1 = 0,
	APDM_DECIMATE_2x1,
	APDM_DECIMATE_2x2,
	APDM_DECIMATE_5x1,
	APDM_DECIMATE_4x2,
	APDM_DECIMATE_5x2,
	APDM_DECIMATE_4x4,
	APDM_DECIMATE_5x4,
	APDM_DECIMATE_8x4,
	APDM_DECIMATE_8x5,
	APDM_DECIMATE_8x8
} apdm_monitor_decimation_rate_t;

typedef enum APDM_Output_Select_Rate {
	APDM_OUTPUT_SELECT_RATE_20 = 0,
	APDM_OUTPUT_SELECT_RATE_32,
	APDM_OUTPUT_SELECT_RATE_40,
	APDM_OUTPUT_SELECT_RATE_64,
	APDM_OUTPUT_SELECT_RATE_80,
	APDM_OUTPUT_SELECT_RATE_128,
	APDM_OUTPUT_SELECT_RATE_256,
	APDM_OUTPUT_SELECT_RATE_512,
	APDM_OUTPUT_SELECT_RATE_640,
	APDM_OUTPUT_SELECT_RATE_1280
} apdm_monitor_output_select_rate_t;

typedef enum APDM_Mode {
	MODE_RESET = 0,
	MODE_RUN = 1,
	MODE_DOCK = 2,
	MODE_ERROR = 3,
	MODE_FULL = 4,
	MODE_OFF = 5,
	MODE_STANDBY = 6,
	MODE_RUN_WAIT = 7,
	MODE_DOCK_WAIT = 8,
	MODE_WIRELESS_CONFIG = 9,
	MODE_OTHER = 10
} apdm_monitor_mode_t;

typedef enum Spin_Mode {
	SPIN_MODE_NONE = 0,
	SPIN_MODE_HALT,
	SPIN_MODE_STANDBY,
	SPIN_MODE_HOLD
} apdm_monitor_spin_mode_t;	

/** @deprecated Will be removed after March 2011 */
typedef apdm_monitor_mode_t apdm_device_mode_t;

typedef enum {
	RECORDING_MODE_STREAMING = 0,
	RECORDING_MODE_MESH,
	RECORDING_MODE_LOGGING
} apdm_monitor_recording_mode_t;

typedef enum {
	DATA_MODE_STREAMED = 0,
	DATA_MODE_LOGGED
} apdm_monitor_data_mode_t;

typedef enum {
	CONFIG_MAG_SET_RESET_OFF = 0,
	CONFIG_MAG_SET_RESET_CONTINUOUS,
	CONFIG_MAG_SET_RESET_SINGLE
} apdm_config_mag_set_reset_t;
	

typedef enum APDM_Battery_Charge_Status {
	CHARGE_STATUS_OFF = 0, CHARGE_STATUS_CHARGING, CHARGE_STATUS_FULL, CHARGE_STATUS_TEST, CHARGE_STATUS_PRE, CHARGE_STATUS_BULK, CHARGE_STATUS_CONST	
} apdm_monitor_battery_charge_status_t;

/**
 * Bitfield representing possible sensor issues detected by apdm_sensor_test_offsets(). Noise results could be triggered by movement while in the dock, so if there are any noise bits set, the result should be presented as a warning, unless the user is sure there was no movement. Gyro recalibration should be recommended for APDM_BAD_GYROSCOPE_OFFSET and mag recalibration should be recommended for APDM_BAD_MAGNETOMETER_OFFSET
 */
typedef enum {
	APDM_OFFSET_OK = 0,
	APDM_BAD_ACCELEROMETER_RANGE = 1,
	APDM_BAD_ACCELEROMETER_OFFSET = 2,
	APDM_BAD_ACCELEROMETER_NOISE = 4,
	APDM_BAD_GYROSCOPE_RANGE = 8,
    APDM_BAD_GYROSCOPE_OFFSET = 16,
    APDM_BAD_GYROSCOPE_NOISE = 32,
    APDM_BAD_MAGNETOMETER_RANGE = 64,
    APDM_BAD_MAGNETOMETER_OFFSET = 128,
    APDM_BAD_MAGNETOMETER_NOISE = 256,
    APDM_BAD_TEMPERATURE_OFFSET = 512,
    APDM_BAD_TEMPERATURE_NOISE = 1024,
    APDM_BAD_TEMPERATURE_RANGE = 2048
} apdm_device_offset_test_code_t;






//====================================================================
// V2 Enums
//====================================================================
/*
typedef enum {
	APDM_V2_SETTING_LOCAL_TIMEZONE,
	APDM_V2_SETTING_ENABLE_ACCEL,
	APDM_V2_SETTING_ENABLE_GYRO,
	APDM_V2_SETTING_ENABLE_MAG,
	APDM_V2_SETTING_ENABLE_WIRELESS,
	APDM_V2_SETTING_MAG_SET_RESET,
	APDM_V2_SETTING_TEMP_SELECT,
	APDM_V2_SETTING_OUTPUT_RATE,
	APDM_V2_SETTING_DECIMATION_SELECT,
	APDM_V2_SETTING_BYPASS_DECIMATION,
	APDM_V2_SETTING_WIRELESS_PROTOCOL,
	APDM_V2_SETTING_WIRELESS_ADDR_ID,
	APDM_V2_SETTING_WIRELESS_CHANNEL_0,
	APDM_V2_SETTING_WIRELESS_CHANNEL_1,
	APDM_V2_SETTING_WIRELESS_LATENCY,
	APDM_V2_SETTING_WIRELESS_MESH_GROUP,
	APDM_V2_SETTING_WIRELESS_MESH_SIZE,
	APDM_V2_SETTING_ALWAYS_OFF,
	APDM_V2_SETTING_BATTERY_LED,
	APDM_V2_SETTING_BATTERY_CUTOFF,
	APDM_V2_SETTING_EXTEND_LED,
	APDM_V2_SETTING_ENABLE_LED,
	APDM_V2_SETTING_ENABLE_BUTTON,
	APDM_V2_SETTING_BUTTON_MODE,
	APDM_V2_SETTING_DEBUG_MODE,
	APDM_V2_SETTING_DEBUG_LED,
	APDM_V2_SETTING_ENABLE_USB_CDC_ACM_VCOM,

	APDM_V2_DEVICE_ID,
	APDM_V2_CASE_ID,
	APDM_V2_GPIO_DIGITAL_INPUT,
	APDM_V2_GPIO_DIGITAL_OUTPUT,
	APDM_V2_GPIO_ANALOG_INPUT,
	APDM_V2_GPIO_ANALOG_OUTPUT,
	APDM_V2_SYNC_VALUE,
	APDM_V2_EPOCH_MS_OFFSET,
} monitor_setting_t;
*/


//====================================================================
// AP Enums
//====================================================================

typedef enum {
	APDM_AP_GPIO_0 = 0,
	APDM_AP_ANALOG_OUT_0 = 1,
	APDM_AP_ANALOG_IN_0 = 2,
} apdm_ap_gpio_pin_t;

//====================================================================
// Docking Station Enums
//====================================================================
typedef enum RequestedDeviceState {
	APDM_DS_RUN = 0, APDM_DS_HALT = 1
} apdm_monitor_requested_state_t;

/*
typedef enum {
	APDM_DOCK_VENDOR_REQUEST_NOP = 0,
	APDM_DOCK_VENDOR_REQUEST_SUPPORTED_FEATURE_BITMASK,
	APDM_DOCK_VENDOR_REQUEST_SET_SD_MOUNTING_MODE,//valid values from enum Monitor_Docking_Event_Handling_Mode
	APDM_DOCK_VENDOR_REQUEST_SET_MONITOR_UART_MODE,
	APDM_DOCK_VENDOR_REQUEST_DOCK_INFO,
	APDM_DOCK_VENDOR_REQUEST_GET_HARDWARE_VERSION,
	APDM_DOCK_VENDOR_REQUEST_GET_FIRMWARE_VERSION,
	APDM_DOCK_VENDOR_REQUEST_GET_MODE,
	APDM_DOCK_VENDOR_REQUEST_GET_IS_MONITOR_PRESENT,
	APDM_DOCK_VENDOR_REQUEST_GET_DOCKED_MODULE_ID,
	APDM_DOCK_VENDOR_REQUEST_GET_MONITOR_DOCKING_STATUS,
	APDM_DOCK_VENDOR_REQUEST_GET_SD_MOUNTING_STATUS,
	APDM_DOCK_VENDOR_REQUEST_GET_NODE_ID,
	APDM_DOCK_VENDOR_REQUEST_GET_CURRENT_POWER_SOURCE,
	APDM_DOCK_VENDOR_REQUEST_GET_MINIMUM_HOST_LIBRARY_VERSION,
	APDM_DOCK_VENDOR_REQUEST_RESET_INTO,
	APDM_DOCK_VENDOR_REQUEST_GET_CASE_ID_DEPRECATED,
	APDM_DOCK_VENDOR_REQUEST_MISC
//valid values from enum Monitor_Uart_Event_Handling_Mode
} usb_vendor_request_type_t;
*/

enum Monitor_Docking_Event_Handling_Mode {
	ODEHM_AUTO = 0, /**< The dock will automatically handle mounting/unmounting the SD on the motion monitor*/
	ODEHM_MOUNT_SD,/**< Command the dock to mount the SD card */
	ODEHM_UNMOUNT_SD,
/**< Command the dock to unmount the SD card */
};

enum Monitor_Uart_Event_Handling_Mode {
	OUEHM_AUTO = 0, /**< The dock will automatically handle docking/undocking the the motion monitor*/
	OUEHM_ENABLED, /**<Command the dock to dock the motion monitor*/
	OUEHM_DISABLED, /**<Command the dock to un-dock the motion monitor*/
};

//====================================================================
// Host Library Enums
//====================================================================

/** \var APDM_Logging_Level The significance level of a given log entry*/
typedef enum APDM_Logging_Level {
	APDM_LL_ALL, APDM_LL_DEBUG, APDM_LL_INFO, APDM_LL_WARNING, APDM_LL_ERROR, APDM_LL_NONE
} apdm_logging_level_t;

enum APDM_Status {
	APDM_OK = 0,
	APDM_INVALID_PARAM,
	APDM_MISSED_DATA, /**<Indicates that there was data missed in the data stream.*/
	APDM_INITIALIZATION_ERROR,
	APDM_NO_MORE_DATA,
	APDM_FT_COMMUNICATIONS_ERROR,
	APDM_STATUS_ERROR,
	APDM_STATUS_UNKNOWN,
	APDM_MEMORY_ALLOCATION_ERROR,
	APDM_INSUFFICIENT_RESOURCES,
	APDM_COMMUNICATIONS_ERROR,/**<Low level protocol communications error*/
	APDM_NOT_ENOUGH_ACCESS_POINTS,
	APDM_INVALID_FIRMWARE,
	APDM_CANT_RESET_INTO_TARGET_MODE,
	APDM_FIRMWARE,
	APDM_BOOTLOADER, //15
	APDM_INVALID_CONTEXT_HANDLE,
	APDM_CRC16_ERROR,
	APDM_DEVICE_RESPONSE_ERROR,
	APDM_DEVICE_FIRMWARE_TOO_OLD,
	APDM_FILE_ERROR,//20

	APDM_UNSUPPORTED_DEVICE_FIRMWARE_VERSION,//21
	APDM_UNSUPPORTED_DOCKING_STATION_FIRMWARE_VERSION,//22
	APDM_UNSUPPORTED_AP_FIRMWARE_VERSION,//23
	APDM_UNSUPPORTED_CALIBRATION_DATA_VERSION,//24

	APDM_SYNC_LINE_ERROR, //25, /**<Can be returned if the sync line is not plugged in between 2 APs, or there is a wiring or electrical problem with the synchronization line between multiple access points.*/

	APDM_UNKNOWN_SENSOR_ID_ERROR, //26

	APDM_DEVICE_NOT_FOUND, //27, /**< If a device (AP, Dock) cannot be found on the host. This can be caused by hardware being open by other applications, or no hardware matching a given serial number of ID.*/

	APDM_UNSUPPORTED_COMMAND, //28

	APDM_DEVICE_TX_RX_COLLISION_ERROR, //29
	APDM_READ_TIMEOUT_ERROR, //30

	APDM_UNABLE_TO_SYNC_RECORD_HEAD_LIST_ERROR, //31

	APDM_UNKNOWN_CALIBRATION_DATA_STRUCTURE, //32

	APDM_NULL_PARAMETER_ERROR, //33

	APDM_NOT_FOUND, //34

	APDM_UNKNOWN_DEVICE_TYPE = 35, //35

	APDM_NO_CORRELATION_FIFO = 36, //36

	APDM_UNSUPPORTED_DATA_FILE_VERSION = 37, //37

	APDM_CORRUPT_DISK_FIFO = 38, //38

	APDM_DOCK_UNSUPPORTED_HOST_LIBRARY_VERSION = 39, //39, /**<the docking station requires a newer version of host libraries to correctly communicate with it*/
	APDM_AP_UNSUPPORTED_HOST_LIBRARY_VERSION = 40, //40, /**<the AP station requires a newer version of host libraries to correctly communicate with it*/

	APDM_UNEXPECTED_STRUCTURE_VALUE = 41,
	
	APDM_BAD_DATA = 42, //42, /**< Indicates bad data detected during calibration. Either in the calibration data stored on the monitor, or in the raw data stream.*/

	APDM_CORRUPT_CONTEXT = 43, //43 /**< If for some reason a part of the contex appears corrupt, this error code will be returned*/

	APDM_AP_READ_TIMEOUT_ERROR = 44,
	APDM_MONITOR_READ_TIMEOUT_ERROR = 45,
	APDM_DOCK_READ_TIMEOUT_ERROR = 46,


	APDM_DEVICE_RESPONSE_ERROR_CRC_ERROR = 47,
	APDM_DEVICE_RESPONSE_ERROR_INVALID_COMMAND,
	APDM_DEVICE_RESPONSE_ERROR_INVALID_PARAM_SIZE,
	APDM_DEVICE_RESPONSE_ERROR_INVALID_PARAM_VALUE,
	APDM_DEVICE_RESPONSE_ERROR_ERROR_EXECUTING_COMMAND,

	APDM_RECALIBRATION_INSUFFICIENT_DATA,
	APDM_RECALIBRATION_BAD_FIT,
	APDM_RECALIBRATION_INSUFFICIENT_MOVEMENT,

	APDM_FIRMWARE_COMBINATION_ERROR,

	APDM_DATA_FROM_UNEXPECTED_MONITOR_ERROR,
	
	APDM_ORIENTATION_ERROR,

	APDM_UNABLE_TO_PING_MONITOR_ERROR,

	APDM_FIRMWARE_INCOMPATABLE,
	APDM_INVALID_MONITOR_CONFIGURATION,

	APDM_UNABLE_TO_MATCH_EXPECTED_AP_IDS,

	APDM_AP_LIBUSB_ERROR_NO_DEVICE,
	APDM_DOCK_LIBUSB_ERROR_NO_DEVICE,

	APDM_UNSUPPORTED_AP_HANDLE_VERSION,
	APDM_NOT_V1_AP_HANDLE,
	APDM_NOT_V2_AP_HANDLE,
	APDM_NOT_V1_DOCK_HANDLE,
	APDM_NOT_V2_MONITOR_HANDLE,
	APDM_HANDLE_VERSION_UNKNOWN,
	APDM_CANT_MIX_HARDWARE_VERSIONS, /**Cant mix versions of hardware*/
	APDM_CORRELATION_ERROR,
	APDM_NOT_ENOUGH_MONITORS,

    APDM_NO_FILE_OVERLAP,
    APDM_ECHO_SYNC_ERROR,
    APDM_AP_NOT_READY_TO_STREAM,
    APDM_INTERPOLATION_ERROR,

    APDM_ASYNC_MODE_RESPONSE,
    APDM_COMMUNICATION_MAGIC_NUMBER_ERROR,

    APDM_SENSOR_RESET_DURING_STREAMING,

    APDM_HDF_ERROR,
    APDM_BUFFER_VALIDATION,
    APDM_DECOMPRESSION_ERROR,

    APDM_DATA_COMM_CRC_ERROR,
    APDM_INCORRECT_NUMBER_OF_APS_CONNECTED,

	APDM_LIB_USB_ERROR = 2000,
	APDM_LIBUSB_ERROR_IO = 2001,/**< Input/output error */
	APDM_LIBUSB_ERROR_INVALID_PARAM = 2002,/**< Invalid parameter */
	APDM_LIBUSB_ERROR_ACCESS = 2003,/**< Access denied (insufficient permissions) */
	APDM_LIBUSB_ERROR_NO_DEVICE = 2004,/**< No such device (it may have been disconnected) */
	APDM_LIBUSB_ERROR_NOT_FOUND = 2005,/**< Entity not found */
	APDM_LIBUSB_ERROR_BUSY = 2006,/**< Resource busy */
	APDM_LIBUSB_ERROR_TIMEOUT = 2007,/**< Operation timed out */
	APDM_LIBUSB_ERROR_OVERFLOW = 2008,/**< Overflow */
	APDM_LIBUSB_ERROR_PIPE = 2009,/**< Pipe error */
	APDM_LIBUSB_ERROR_INTERRUPTED = 2010,/**< System call interrupted (perhaps due to signal) */
	APDM_LIBUSB_ERROR_NO_MEM = 2011,/**< Insufficient memory */
	APDM_LIBUSB_ERROR_NOT_SUPPORTED = 2012,/**< Operation not supported or unimplemented on this platform */
	APDM_LIBUSB_ERROR_UNKOWN = 2098,
	APDM_LIBUSB_ERROR_OTHER = 2099,/**< Other error */

	APDM_FTDI_COMM_ERROR = 3000,
};

enum APDM_Status_Severity {
	APDM_SEVERITY_ERROR, APDM_SEVERITY_WARNING, APDM_SEVERITY_INFO
};

enum APDMFieldTypes {
	ACCELEROMETERS = 1, GYROS = 2, TEMPERATURE = 3, MAGNETOMETER = 4, DEPRECATED = 5, EVENTS = 6, DEVICE_INFORMATION = 7
};

/**Error Handling Behavior, used to describe how to deal with partial records and dropped packets during data streaming.*/
enum APDMErrorHandlingBehavior {
	APDME_EHB_OMIT_RECORD = 0, APDME_EHB_RETURN_PARTIAL_RECORD
};

/**
 * This enum defines the valid types for data in the optional data field of a raw sensor sample.
 */
typedef enum {
	MONITOR_OPT_SELECT_NONE,
	MONITOR_OPT_SELECT_VBAT,
	MONITOR_OPT_SELECT_MSP_TEMP,
	MONITOR_OPT_SELECT_IDG_TEMP,
	MONITOR_OPT_SELECT_SAMPLES,
	MONITOR_OPT_SELECT_TEMP_SUM,
	MONITOR_OPT_SELECT_TAG_DATA,
	MONITOR_OPT_SELECT_MODULE_ID
} apdm_raw_opt_select_t;

typedef enum APDM_Temp_Sensor_Select {
	APDM_TEMP_SENSOR_MSP = 0,
	APDM_TEMP_SENSOR_GYRO = 1
} apdm_monitor_temp_sensor_select_t;

typedef struct {
	apdm_monitor_error_id_t error_id;
	uint32_t error_count;
	uint64_t sync_value;/**< A non-zero sync value means that this has valid data*/
} apdm_monitor_error_stat_t;

typedef enum {
	APDM_AP_RECEIVING_UNKNOWN,
	APDM_AP_RECEIVING_FROM_ALL_MONITORS,
	APDM_AP_RECEIVING_FROM_ALL_MONITORS_AND_CATCHING_UP,
	APDM_AP_RECEIVING_FROM_ALL_MONITORS_AND_FALLING_BEHIND,
	APDM_AP_NOT_RECEIVING_DATA_FROM_SOME_MONITORS,
	APDM_AP_NOT_RECEIVING_DATA_FROM_ANY_MONITORS
} apdm_ap_wireless_streaming_status_t;

typedef enum {
    APDM_ORIENTATION_MODEL_ALL = 0,
    APDM_ORIENTATION_MODEL_NO_MAG = 1,
    APDM_ORIENTATION_MODEL_UNDISTURBED_MAG = 2
} apdm_orientation_model_t;

typedef enum {
    APDM_FILE_VERSION_UNKNOWN,
    APDM_FILE_VERSION_0000,
    APDM_FILE_VERSION_0001,
    APDM_FILE_VERSION_0002,
    APDM_FILE_VERSION_0003,
    APDM_FILE_VERSION_0004,
    APDM_FILE_VERSION_0005,
    APDM_FILE_VERSION_0006,
    APDM_FILE_VERSION_0007,
    APDM_FILE_VERSION_2001,
}apdm_file_version_t;


/**
 * check for a specific error using the enum position to right shift the data_status 
 * if ((data_staus >> APDM_INTEGRITY_ACC_STUCK) & 1)
 */
typedef enum {
	APDM_DATA_INTEGRITY_GOOD = 0,
	APDM_DATA_INTEGRITY_ACC_STUCK,
	APDM_DATA_INTEGRITY_ACC_NAN,
	APDM_DATA_INTEGRITY_ACC_INF,
	APDM_DATA_INTEGRITY_ACC_NO_DATA,
	APDM_DATA_INTEGRITY_GYRO_STUCK,
	APDM_DATA_INTEGRITY_GYRO_NAN,
	APDM_DATA_INTEGRITY_GYRO_INF,
	APDM_DATA_INTEGRITY_GYRO_NO_DATA,
	APDM_DATA_INTEGRITY_MAG_STUCK,
	APDM_DATA_INTEGRITY_MAG_NAN,
	APDM_DATA_INTEGRITY_MAG_INF,
	APDM_DATA_INTEGRITY_MAG_NO_DATA,
	APDM_DATA_INTEGRITY_PRESSURE_STUCK,
	APDM_DATA_INTEGRITY_PRESSURE_NAN,
	APDM_DATA_INTEGRITY_PRESSURE_INF,
	APDM_DATA_INTEGRITY_PRESSURE_NO_DATA,
	APDM_DATA_INTEGRITY_ORIENTATION_NAN,
	APDM_DATA_INTEGRITY_OUT_OF_ORDER,
	APDM_DATA_INTEGRITY_SKIPPED_SAMPLE,

	APDM_DATA_INTEGRITY_ACC_RANGE,
	APDM_DATA_INTEGRITY_GYRO_RANGE,
	APDM_DATA_INTEGRITY_MAG_RANGE,
	APDM_DATA_INTEGRITY_PRESSURE_RANGE,

	APDM_DATA_INTEGRITY_LAST_ELEMENT
} apdm_data_integrity_bitfield_t;


/**
 * The APDM context is a logical grouping of one or more AP's and one or more Monitors, and deals with the correct
 * configuration of the system, streaming and synchronization of data off the AP's and other things that you might
 * want do on a system as a whole.
 */
typedef void* apdm_ctx_t;

/**@deprecated This is no longer used, replaced by apdm_ctx_t, will be removed after January 2011*/
typedef apdm_ctx_t apdm_device_communications_handle_new_t;

/**This is a handle used to manipulate an individual access point */
typedef void* apdm_ap_handle_t;

/**This is a handle used to manipulate a single monitor/docking station */
typedef void* apdm_device_handle_t;

/**@deprecated This should no longer be used and will be removed after January 2011*/
typedef apdm_device_handle_t apdm_dockingstation_handle_t;

/** */
typedef void* apdm_csv_t;

/**
 * FIXME document this
 */
typedef struct {
    int result_code;
    uint8_t gyro_recalibration_block[CALIBRATION_DATA_BUFFER_SIZE];
    enum APDM_Status gyro_recalibration_result;
    uint32_t sd_mbytes_total; /**<  Currently filled in by MotionStudio*/
    uint32_t sd_mbytes_used; /**<  Currently filled in by MotionStudio*/
} apdm_device_status_t;


/**
 * APDM Sensor Sample
 */
typedef struct {

	uint64_t sync_val64;/**< Full 64 bit sync value*/
	uint64_t v2_sync_val64_us;/**< V2 microsecond sync value. For V1 hardware samples, this will be the epoch microsecond*/
	uint64_t v2_mcu_time_val64_us;/**< V2 mcu microsecond time.*/
	int64_t v2_sync_time_delta_ms;
	uint32_t sync_val32_low;//@deprecated This will go away after Jan 2011, use the 64 bit sync value instead
	uint32_t sync_val32_high;//@deprecated This will go away after Jan 2011, use the 64 bit sync value instead
	uint8_t nRF_pipe;/**< Internal use only \internal */
	uint8_t num_retrys;/**< Number of retry before this sample was received by the AP. */
	int32_t source_ap_index;/**< Index of the AP that the sample came in on. */
	uint16_t accl_x_axis;/**<raw ADC readings*/
	uint16_t accl_y_axis;/**<raw ADC readings*/
	uint16_t accl_z_axis;/**<raw ADC readings*/

  int16_t accl_high_x_axis;
  int16_t accl_high_y_axis;
  int16_t accl_high_z_axis;

  bool accl_full_scale_mode;/**<True indicates accelerometers are in 6G mode, false indicates 2G mode*/
	bool accl_isPopulated;/**<Indicates that the accel data is populated*/

	uint16_t gyro_x_axis;/**<raw ADC readings*/
	uint16_t gyro_y_axis;/**<raw ADC readings*/
	uint16_t gyro_z_axis;/**<raw ADC readings*/
	bool gyro_isPopulated;/**<Indicates that the gyro data is populated*/

	uint16_t mag_x_axis;/**<raw ADC readings*/
	uint16_t mag_y_axis;/**<raw ADC readings*/
	uint16_t mag_z_axis;/**<raw ADC readings*/
	uint16_t mag_common_axis;/**< @depreciated only used with device protocol version 0. raw ADC readings*/
	bool mag_isPopulated;/**<Indicates that the mag data is populated*/

	uint8_t flag_accel_enabled;
	uint8_t flag_gyro_enabled;
	uint8_t flag_mag_enabled;
	uint8_t flag_full_scale_enabled;
	uint8_t flag_sync_lock;
	uint8_t flag_sync_reset;
	uint8_t flag_temp_select;

	uint16_t flags;/**<Flags packed binary structure, used to derive the flag_XXXX field values.*/
	bool gyro_temperature_sensor_selected;

	uint32_t optional_data;/**<Optional and varying data from the monitor*/
	uint8_t opt_select;/**<Indicates what type of data is in optional_data, see enum apdm_raw_opt_select_t*/

	uint32_t debug_data;
	uint16_t debug_flags;
	
	double temperature;
	double temperature_average;
	double temperature_diff;
	bool temperature_isPopulated;

	uint32_t batt_voltage;/**<raw ADC readings*/
	bool batt_voltage_isPopulated;/**<Indicates that the battery voltage data is populated*/

	uint32_t device_info_serial_number;/**< Device ID */
	uint8_t device_info_wireless_channel_id;/**< */
	uint8_t device_info_wireless_address;/**< */
	bool device_info_isPopulated;/**< */

	uint8_t button_status;
	
	uint32_t tag_data;
	bool tag_data_isPopulated;

	uint32_t sensor_version; //of type apdm_ap_handle_version_t

	//S.I. Converted Data
	double accl_x_axis_si;/** Fused accelerometer <meters per second^2*/
	double accl_y_axis_si;/** Fused accelerometer <meters per second^2*/
	double accl_z_axis_si;/** Fused accelerometer <meters per second^2*/


	double gyro_x_axis_si;/**<radians per second*/
	double gyro_y_axis_si;/**<radians per second*/
	double gyro_z_axis_si;/**<radians per second*/

	double mag_x_axis_si;/**< a.u. */
	double mag_y_axis_si;/**< a.u. */
	double mag_z_axis_si;/**< a.u. */
    
    double gyro_x_axis_filtered;
    double gyro_y_axis_filtered;
    double gyro_z_axis_filtered;
	
	double orientation_quaternion0; /* Orientation relative to the NWU local tangent plane with N being defined as magnetic north*/
	double orientation_quaternion1; /* Orientation relative to the NWU local tangent plane with N being defined as magnetic north*/
	double orientation_quaternion2; /* Orientation relative to the NWU local tangent plane with N being defined as magnetic north*/
	double orientation_quaternion3; /* Orientation relative to the NWU local tangent plane with N being defined as magnetic north*/

	double temperature_si;/**<degrees celcius*/
	double temperature_derivative_si;/**<degrees C per sec */

	uint16_t pressure;/** */
	double pressure_si;/** Pascals*/
	bool pressure_isPopulated;

	double accl_low_x_axis_si;/** Low range accelerometer <meters per second^2*/
	double accl_low_y_axis_si;/** Low range accelerometer <meters per second^2*/
	double accl_low_z_axis_si;/** Low range accelerometer <meters per second^2*/

	double accl_high_x_axis_si;/** High range accelerometer <meters per second^2*/
	double accl_high_y_axis_si;/** High range accelerometer <meters per second^2*/
	double accl_high_z_axis_si;/** High range accelerometer <meters per second^2*/


	double battery_level;/**< percent */

	uint32_t data_status; // 0 if data is good, otherwise set bit positions have meaning according to apdm_data_integrity_bitfield_t

} apdm_record_t;

typedef struct {
	uint64_t sync_val64; /**< V1 sync value.*/

	uint32_t source_ap_index;
	uint64_t sync_epoch_ms_offset; //Number of ms since Jan 1, 1970
	uint64_t wireless_sync_time_us;
	uint64_t microcontroller_time_us;
	uint32_t flags;

	uint64_t stm32_time_us;
	uint32_t source_device_id;
	uint32_t remote_device_id;
	uint64_t anchor_timestamp;
} apdm_ranging_sample_t;

typedef struct {
	uint8_t data;            /**< For V2 hardware: zero if the I/O pin is low (zero volts), 1 if the I/O pin is high. */
	uint8_t data_type;       /**< For V1 hardware: value is from enum External_Sync_Data_Types. For V2 hardware: value from the enum can_cmd_types_t. */
	uint64_t sync_value;     /**< Time at which the event occurred. */
	uint64_t sync_value_v2;  /**< Time at which the event occurred for V2 hardware. */
	uint32_t ap_id;          /**< ID of the access point that the event occurred on. */
	uint32_t v2_pin;         /**< For V2 hardware: of type can_sync_box_pin_t */
} apdm_external_sync_data_t;

typedef struct {
	double state[3];/**< initialize to [0 0 0]; */
	double state_transition_matrix[9];/**< initialize to [1 0 0; 0 1 0; 0 0 1] */
	double process_noise_matrix[9];/**< */
	double measurement[3];/**< initialize to [0;0;0]; */
	double measurement_matrix[9];/**< */
	double measurement_noise_matrix[9];/**< */
	double error_covariance_matrix[9];/**< */
	double filtered_measurement[3];/**< */
} apdm_mag_step_response_state_t;/** */

typedef struct {
	double state[2];/**< initialize to [0 0]; */
	double state_transition_matrix[4];/**< initialize to [0 0; 0 1] */
	double process_noise_matrix[4];/**< */
	double measurement[6];/**< initialize to [0;0;0;0;0;0;0]; */
	double measurement_matrix[12];/**< */
	double measurement_noise_matrix[6 * 6];/**< */
	double error_covariance_matrix[4];/**< */
	double filtered_measurement[6];/**< */
	double stepResponse[10];/**< */
	apdm_mag_step_response_state_t stepResponseEstimate;
	int set_reset_flag;/**< */
	int polarity;/**< */
	int iSample;/**< */
} apdm_mag_dechop_state_t;/**< */
	
	
	
#define APDM_RECORD_INITIALIZER (apdm_record_t) { \
	.sync_val64 = 0, \
	.sync_val32_low = 0, \
	.sync_val32_high = 0, \
	.num_retrys = 0, \
	.accl_x_axis = 0, \
	.accl_y_axis = 0, \
	.accl_z_axis = 0, \
	.accl_isPopulated = false, \
	.gyro_x_axis = 0, \
	.gyro_y_axis = 0, \
	.gyro_z_axis = 0, \
	.gyro_isPopulated = false, \
	.mag_x_axis = 0, \
	.mag_y_axis = 0, \
	.mag_z_axis = 0, \
	.mag_common_axis = 0, \
	.flags = 0, \
	.gyro_temperature_sensor_selected = true, \
	.mag_isPopulated = false, \
    .temperature = 0, .temperature_average = 0, .temperature_isPopulated = false, \
    .device_info_serial_number = 0, .device_info_wireless_channel_id = 0, \
    .device_info_wireless_address = 0, \
    .device_info_isPopulated = false, \
    .accl_x_axis_si = 0.0, .accl_y_axis_si = 0.0, .accl_z_axis_si = 0.0, \
    .gyro_x_axis_si = 0.0, .gyro_y_axis_si = 0.0, .gyro_z_axis_si = 0.0, .mag_x_axis_si = 0.0, \
    .mag_y_axis_si = 0.0, .mag_z_axis_si = 0.0, .temperature_si = 0.0, .temperature_derivative_si = 0.0, \
    .battery_level = 0.0\
}

typedef struct {
	double accl_x_bias;/**< */
	double accl_y_bias;/**< */
	double accl_z_bias;/**< */
	double accl_x_bias_temp;/**< */
	double accl_y_bias_temp;/**< */
	double accl_z_bias_temp;/**< */
	double accl_z_bias_dtemp;/**< */
	double accl_x_scale;/**< */
	double accl_y_scale;/**< */
	double accl_z_scale;/**< */
	double accl_x_scale_temp;/**< */
	double accl_y_scale_temp;/**< */
	double accl_z_scale_temp;/**< */
	double accl_xy_sensitivity;/**< */
	double accl_xz_sensitivity;/**< */
	double accl_yz_sensitivity;/**< */
	double accl_error_matrix[3 * 3];/**< this will be calculated by the host libraries */

	double gyro_x_bias;/**< */
	double gyro_y_bias;/**< */
	double gyro_z_bias;/**< */
	double gyro_x_bias_temp;/**< */
	double gyro_x_bias_temp2;/**< */
	double gyro_y_bias_temp;/**< */
	double gyro_y_bias_temp2;/**< */
	double gyro_z_bias_temp[61];/**< */
	double gyro_x_scale;/**< */
	double gyro_y_scale;/**< */
	double gyro_z_scale;/**< */
	double gyro_x_scale_temp;/**< */
	double gyro_y_scale_temp;/**< */
	double gyro_z_scale_temp;/**< */
	double gyro_xy_sensitivity;/**< */
	double gyro_xz_sensitivity;/**< */
	double gyro_yz_sensitivity;/**< */
	double gyro_accl_roll;/**< */
	double gyro_accl_pitch;/**< */
	double gyro_accl_yaw;/**< */
	double gyro_error_matrix[3 * 3];/**< this will be calculated by the host libraries */

	double mag_x_bias;/**< */
	double mag_y_bias;/**< */
	double mag_z_bias;/**< */
	double mag_x_scale;
	double mag_y_scale;
	double mag_z_scale;
	double mag_xy_sensitivity;
	double mag_xz_sensitivity;
	double mag_yz_sensitivity;
	double mag_accl_roll;
	double mag_accl_pitch;
	double mag_accl_yaw;
	double mag_error_matrix[3 * 3];

	apdm_mag_dechop_state_t mag_x_state;/**< */
	apdm_mag_dechop_state_t mag_y_state;/**< */
	apdm_mag_dechop_state_t mag_z_state;/**< */

	double temperature_bias;/**< */
	double temperature_scale;/**< */

	double temperature_bias_msp;/**< */
	double temperature_scale_msp;/**< */
} calibration_v4_t;

typedef struct {
	uint16_t accl_x_bias[61];/**< Temperature dependent bias covering the range [-10,50] C */
	uint16_t accl_y_bias[61];/**< Temperature dependent bias covering the range [-10,50] C */
	uint16_t accl_z_bias[61];/**< Temperature dependent bias covering the range [-10,50] C */
	double accl_z_bias_dtemp;/**< */
	double accl_x_scale;/**< */
	double accl_y_scale;/**< */
	double accl_z_scale;/**< */
	double accl_x_scale_temp;/**< */
	double accl_y_scale_temp;/**< */
	double accl_z_scale_temp;/**< */
	double accl_xy_sensitivity;/**< */
	double accl_xz_sensitivity;/**< */
	double accl_yz_sensitivity;/**< */
	double accl_error_matrix[3 * 3];/**< this will be calculated by the host libraries*/
	
	uint16_t gyro_x_bias[61];/**< */
	uint16_t gyro_y_bias[61];/**< */
	uint16_t gyro_z_bias[61];/**< */
	double gyro_x_scale;/**< */
	double gyro_y_scale;/**< */
	double gyro_z_scale;/**< */
	double gyro_x_scale_temp;/**< */
	double gyro_y_scale_temp;/**< */
	double gyro_z_scale_temp;/**< */
	double gyro_xy_sensitivity;/**< */
	double gyro_xz_sensitivity;/**< */
	double gyro_yz_sensitivity;/**< */
	double gyro_accl_roll;/**< */
	double gyro_accl_pitch;/**< */
	double gyro_accl_yaw;/**< */
	double gyro_error_matrix[3 * 3];/**< this will be calculated by the host libraries*/
	
	uint16_t mag_x_bias[61];/**< */
	uint16_t mag_y_bias[61];/**< */
	uint16_t mag_z_bias[61];/**< */
	double mag_x_scale;
	double mag_y_scale;
	double mag_z_scale;
	double mag_x_scale_temp;
	double mag_y_scale_temp;
	double mag_z_scale_temp;
	double mag_xy_sensitivity;
	double mag_xz_sensitivity;
	double mag_yz_sensitivity;
	double mag_accl_roll;
	double mag_accl_pitch;
	double mag_accl_yaw;
	
	double mag_x_offset; /**< Used internally for removing set/reset pulse artifacts*/
	double mag_y_offset; /**< Used internally for removing set/reset pulse artifacts*/
	double mag_z_offset; /**< Used internally for removing set/reset pulse artifacts*/
	
	double mag_conversion_gain;
	
	double mag_error_matrix[3 * 3];
	
	apdm_mag_dechop_state_t mag_x_state;/**< */
	apdm_mag_dechop_state_t mag_y_state;/**< */
	apdm_mag_dechop_state_t mag_z_state;/**< */
	
	double temperature_bias;/**< */
	double temperature_scale;/**< */
	
	double temperature_bias_msp;/**< */
	double temperature_scale_msp;/**< */
	
} calibration_v5_t;
	
typedef struct {
    uint16_t accl_x_bias[61];/**< Temperature dependent bias covering the range [-10,50] C */
    uint16_t accl_y_bias[61];/**< Temperature dependent bias covering the range [-10,50] C */
    uint16_t accl_z_bias[61];/**< Temperature dependent bias covering the range [-10,50] C */
    double accl_z_bias_dtemp;/**< */
    double accl_x_scale;/**< */
    double accl_y_scale;/**< */
    double accl_z_scale;/**< */
    double accl_x_scale_temp;/**< */
    double accl_y_scale_temp;/**< */
    double accl_z_scale_temp;/**< */
    double accl_xy_sensitivity;/**< */
    double accl_xz_sensitivity;/**< */
    double accl_yz_sensitivity;/**< */
    double accl_error_matrix[3 * 3];/**< this will be calculated by the host libraries*/
    
    uint16_t gyro_x_bias[61];/**< */
    uint16_t gyro_y_bias[61];/**< */
    uint16_t gyro_z_bias[61];/**< */
    double gyro_x_scale;/**< */
    double gyro_y_scale;/**< */
    double gyro_z_scale;/**< */
    double gyro_x_scale_temp;/**< */
    double gyro_y_scale_temp;/**< */
    double gyro_z_scale_temp;/**< */
    double gyro_xy_sensitivity;/**< */
    double gyro_xz_sensitivity;/**< */
    double gyro_yz_sensitivity;/**< */
    double gyro_accl_roll;/**< */
    double gyro_accl_pitch;/**< */
    double gyro_accl_yaw;/**< */
    double gyro_error_matrix[3 * 3];/**< this will be calculated by the host libraries*/
    
    uint16_t mag_x_bias;/**< */
    uint16_t mag_y_bias;/**< */
    uint16_t mag_z_bias;/**< */
    double mag_x_scale;
    double mag_y_scale;
    double mag_z_scale;
    double mag_xy_sensitivity;
    double mag_xz_sensitivity;
    double mag_yz_sensitivity;
    double mag_accl_x;
    double mag_accl_y;
    double mag_accl_z;
    
    double mag_x_offset; /**< Used internally for removing set/reset pulse artifacts */
    double mag_y_offset; /**< Used internally for removing set/reset pulse artifacts */
    double mag_z_offset; /**< Used internally for removing set/reset pulse artifacts */
    
    double mag_conversion_gain; /**< Calibrated to magnitude 1, this is to convert to uT and should be equal to the field strength during calibration */
    double mag_inclination; /**< Inclination angle in rad. pi/2 - ang(mag, -gravity)*/
    double mag_error_matrix[3 * 3];
    
    apdm_mag_dechop_state_t mag_x_state;/**< */
    apdm_mag_dechop_state_t mag_y_state;/**< */
    apdm_mag_dechop_state_t mag_z_state;/**< */
    
    double temperature_bias;/**< */
    double temperature_scale;/**< */
    
    double temperature_bias_msp;/**< */
    double temperature_scale_msp;/**< */
    
} calibration_v6_t;
	
    
typedef enum {
	CALIBRATION_V4 = 4, 
    CALIBRATION_V5 = 5, 
    CALIBRATION_V6 = 6,
    CALIBRATION_V7 = 7
} calibration_versions_t;

/** */
typedef struct {
	uint32_t converted_calibration_version;
	uint32_t raw_calibration_version;
	union {
		//Only one of these structures is relevant at any given point in time.
		calibration_v4_t v4;
		calibration_v5_t v5;
        calibration_v6_t v6;
        calibration_data_t v7;
	} data;
} apdm_sensor_compensation_t;/**< */
    
typedef struct {
	//User-Configurable settings on the motion monitor
	bool decimation_bypass_flag;
	bool time_good_flag;
	bool accelerometer_full_scale_flag;
	bool accelerometer_enabled_flag;
	bool gyroscope_enabled_flag;
	bool magnetometer_enabled_flag;
	bool pressure_enabled_flag;
	bool ranging_enabled_flag;
	bool sd_card_enabled_flag;
	bool always_off_flag;
	bool erase_sd_card_after_undocking;/**< True if you want the monitor to erase the SD card after it undocks*/
	bool enable_button;
	uint8_t button_mode;

	bool battery_led;
	uint32_t extend_led;
	bool debug_led;
	uint32_t battery_cutoff;
	uint32_t wireless_latency;


	apdm_monitor_spin_mode_t spin_mode;
	uint8_t selected_temperature_sensor;/**<  (1:APDM_TEMP_SENSOR_GYRO or 0:APDM_TEMP_SENSOR_MSP)*/
	apdm_monitor_decimation_rate_t decimation_rate;
	//apdm_monitor_output_select_rate_t output_select_rate;
	uint16_t sample_rate;/**< E.G. 128, can be directly derived from output_select_rate, this should be set with results from apdm_monitor_output_select_rate_t_to_int() */
	uint32_t decimation_factor;/**< E.G. 10, can be directly derived from decimation_rate, this should be set with results from apdm_monitor_decimation_rate_t_to_int() */
	int32_t timezone; /**< offset from UTC in minutes such that local time = UTC + timezone*/
	char device_label[DEVICE_LABEL_SIZE];
    apdm_orientation_model_t orientation_model; /**< Orientation model to use*/


	//Read-only values from the motion monitor
	uint8_t calibration_binary_blob[CALIBRATION_DATA_BUFFER_SIZE];
	uint32_t calibration_version_number;

	uint8_t user_calibration_binary_blob[CALIBRATION_DATA_BUFFER_SIZE];
	uint32_t user_calibration_version_number;

	uint32_t device_id;
	uint32_t hardware_id;

	char sd_file_version[9];
	char firmware_version_string1[VERSION_STRING_SIZE];
	char firmware_version_string2[VERSION_STRING_SIZE];
	int64_t firmware_version_string2_number;
	char firmware_version_string3[VERSION_STRING_SIZE];
	char case_id[CASE_ID_SIZE];
    char timezone_string[TIMEZONE_STRING_SIZE];

	apdm_config_mag_set_reset_t magnetometer_set_reset; // 0: CONFIG_MAG_SET_RESET_OFF, 1: CONFIG_MAG_SET_RESET_CONTINUOUS, 2: CONFIG_MAG_SET_RESET_SINGLE
	apdm_monitor_recording_mode_t recording_mode;
	apdm_monitor_data_mode_t data_mode;


	bool enable_wireless;
	apdm_wireless_mode_t wireless_protocol;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	wireless_v2_radio_mode_t wireless_protocol_v2;
	uint8_t wireless_timeslice;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	uint8_t wireless_addr_id;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.
	                             Defines the pipe-number that data will come in on for the monitor.  */

	uint32_t protocol_version;

	uint8_t wireless_channel0;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	uint32_t wireless_block0;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	uint8_t wireless_channel1;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	uint32_t wireless_block1;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	uint8_t wireless_channel2;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	uint32_t wireless_block2;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	uint8_t wireless_channel3;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/
	uint32_t wireless_block3;/**<CAUTION: modifying this can cause unpredictable behavior, allow autoconfigure() or other similar functions to set this value.*/

	uint32_t dock_id_during_configuration;
	uint32_t dock_hardware_version_during_configuration;
	monitor_configuration_t v2_config;
	uint32_t ap_rx_rssi;
	uint32_t v2_device_statistics[DEVICE_STATISTIC_LAST_ELEMENT];
	ap_sensor_status_data_t ap_sensor_status_data;


	bool have_ss_data;

	uint64_t last_received_compressed_stm32_time_us;
	char v2_device_location[DEVICE_LABEL_SIZE];
} apdm_device_info_t;

typedef struct {
  ap_sensor_status_data_t ap_sensor_status_data;
} apdm_streaming_status_t;


typedef struct {
	uint8_t wireless_channel_number; ;/**<The base wireless channel number to use*/
	bool enable_sd_card; ;/**<Boolean indicating weather or not data should be logged to the SD card on the device.*/
	bool erase_sd_card; ;/**<Boolean flag indicating that the data on the SD card should be erased as part of the initialization process.*/
	bool accel_full_scale_mode; ;/**<If true, then accelerometers will be in 6G mode, if false, then they will be in 2G mode*/
	bool enable_accel; ;/**<Enable the accelerometers*/
	bool enable_gyro; ;/**<Enable the gyros*/
	bool enable_mag; ;/**<Enable the magnitometers*/
	bool enable_pressure; /**<Enable the pressure sensor*/
	bool apply_new_sensor_modes; ;/**<If set to true, flags are carried thru to the device_info structure*/
	bool set_configuration_on_device; ;/**< Allows you to disable the setting of the configuration on the device so that it can be done later in a threaded/concurent manor by the application.*/
	apdm_monitor_decimation_rate_t decimation_rate; ;/**<*/
	//apdm_monitor_output_select_rate_t output_select_rate; ;/**<select output rate*/
	uint32_t output_rate_hz;
  uint32_t wireless_divider; ;/**For v2 monitors, stream data using this divider relative to the logging sample rate**/
	bool button_enable; ;/**<Enable monitor button accessory*/
	uint32_t wireless_max_latency_ms;  /**< Maximum latency for data during wireless transmission.*/
	uint32_t wireless_group_code;
	bool wireless_rapid_streaming;
	apdm_device_info_t device_info_cache[APDM_MAX_NUMBER_OF_SENSORS];
} apdm_streaming_config_t;



typedef struct {
	apdm_device_info_t device_info;
	uint64_t start_sync_count;
	uint64_t end_sync_count;
	uint64_t epoch_time_offset_us;
    int num_samples;
} apdm_recording_info_t;
	
//typedef struct {
//	apdm_recording_info_t *monitor_info;
//	int nMonitors;
//	int hdf_format_version;
//	
//} apdm_hdf_file_info_t;

typedef struct {
	uint64_t time;//epoch microseconds
	uint32_t device_id;
	char text[2048];
} apdm_annotation_t;

typedef struct {
    uint64_t time;
    uint32_t error_id;
    char name[32];
    uint32_t value;
} apdm_error_table_t;

typedef struct {
    uint32_t config_id;
    char name[128];
    uint32_t value;
} apdm_v2_config_table_t;

typedef struct {
    uint32_t config_string_id;
    char name[128];
    char value[128];
} apdm_v2_config_string_table_t;

typedef struct {
    char name[128];
    char value[128];
} apdm_v2_config_misc_table_t;

typedef struct {
	char task[128];
	int num_tasks;
	int task_index;
	double percent_complete; /**< 0-100 floating point for current task*/
} apdm_progress_t;

typedef struct {
	char label[DEVICE_LABEL_SIZE];
} apdm_monitor_label_t;

typedef struct {
	char id[CASE_ID_SIZE];
} apdm_case_id_t;

typedef struct {
	char *file;
	double local_field_magnitude;
	uint8_t calibration_block[2048];
	double original_calibrated_data[115200];
	double updated_calibrated_data[115200];
	int num_samples;
}apdm_magnetometer_recalibration_t;
    
typedef struct {
    //Required parameters
    char **files_to_convert;          /**< array of .apdm file name paths*/
    int nFiles;               /**< number of files to convert (size of files_in array)*/
    const char *file_out;           /**< output file path*/
    
    //Optional parameters
    bool store_raw;           /**< true to store raw ADC data in the output file */
    bool store_si;            /**< true to store calibrated data in SI units */
    bool store_filtered;      /**< true to store filtered calibrated data */
    bool format_hdf;          /**< true to store output in HDF5 format, false to store output in CSV format */
    bool compress;            /**< true to compress HDF data, has no effect if output is CSV format */
    char csv_delimiter;       /**< delimiter character to use for csv files. Default is ','. Has no effect if output is HDF format */
    apdm_progress_t *progress;/**< progress structure updated during the file conversion process that can be inspected by another thread for updating a progress bar */
    uint64_t sync_start;      /**< only data after sync_start will be included in the output file, default 0 indicates all data included */
    uint64_t sync_end;        /**< no data after sync_end will be included in the output file, default 0 indicates all data included */
    uint64_t epoch_time_offset_us; /**< offset to convert from synchornized recording timestamps to epoch timestamps */
    char timezone_string[TIMEZONE_STRING_SIZE]; /**< Timezone string (eg. "America/Los_Angeles") */
    
    //These are mostly used for internal development purposes and should not be modified from defaults
    bool dechop_raw_magnetometer; /**< default true */
    char **calibration_files; /**< default NULL to indicate calibration parameters included in each file should be used */
    apdm_orientation_model_t orientation_model;
    bool store_unsynchronized;  /**< true to store synchronized data (v2). false to store unsynchronized data (v2). No effect (v1). */
    bool store_all_sensors; /**< default false. true will additionally store low range and high range accelerometer data in separate datasets (v2)*/
}apdm_file_conversion_parameter_t;


typedef struct {
  button_event_data_t device_button_data;

  uint64_t unix_epoch_second; /**<Number of seconds since 1970 */
  uint64_t sync_val64; /**<Sync value in units of V1 ap sync values */
} apdm_button_data_t;



	
#ifdef __cplusplus
}
#endif

#endif /*APDM_TYPES_H_*/
