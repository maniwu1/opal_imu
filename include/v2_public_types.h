#ifndef V2_PUBLIC_TYPES_H_
#define V2_PUBLIC_TYPES_H_

#define APDM_CONFIG_V2_STRING_MAX_LENGTH    24

typedef enum {
  CONFIG_V2_STRING_LABEL_0 = 0,
  CONFIG_V2_STRING_LABEL_1,
  CONFIG_V2_STRING_LABEL_2,
  CONFIG_V2_STRING_LOCATION,
  CONFIG_V2_STRING_TIMEZONE,
  CONFIG_V2_STRING_BUTTON_EVENT_0,
  CONFIG_V2_STRING_BUTTON_EVENT_1,
  CONFIG_V2_STRING_BUTTON_EVENT_2,
  CONFIG_V2_STRING_BUTTON_EVENT_3,
  CONFIG_V2_STRING_TIMEZONE_OFFSET,

  CONFIG_V2_STRING_LAST_VALUE
} config_v2_string_t;

typedef enum {
	CALIBRATION_TYPE_DEFAULT = 0,
	CALIBRATION_TYPE_USER,
	CALIBRATION_TYPE_SX
} calibration_type_t;


#define CALIBRATION_BIAS_ARRAY_SIZE       61
#define CALIBRATION_MISALIGNMENT_ARRAY_SIZE     9

typedef struct {
  uint32_t length;
  uint32_t structure_version;
  uint32_t device_id; //Sanity check to make sure that people are loading the correct file
  union {
    struct {
      int16_t acc_x_bias[CALIBRATION_BIAS_ARRAY_SIZE]; //temperature dependent bias -10C to 50C in 1 deg increments
      int16_t acc_y_bias[CALIBRATION_BIAS_ARRAY_SIZE];
      int16_t acc_z_bias[CALIBRATION_BIAS_ARRAY_SIZE];
      uint8_t pad0;
      float acc_scale[3];
      float acc_scale_temp[3];
      float acc_xy_sensitivity;
      float acc_xz_sensitivity;
      float acc_yz_sensitivity;
      float acc_roll;
      float acc_pitch;
      float acc_yaw;
      float acc_scale_misalignment[CALIBRATION_MISALIGNMENT_ARRAY_SIZE];
      float gravity_magnitude;

      int16_t accH_x_bias[CALIBRATION_BIAS_ARRAY_SIZE]; //temperature dependent bias -10C to 50C in 1 deg increments
      int16_t accH_y_bias[CALIBRATION_BIAS_ARRAY_SIZE];
      int16_t accH_z_bias[CALIBRATION_BIAS_ARRAY_SIZE];
      uint8_t pad1;
      float accH_scale[3];
      float accH_scale_temp[3];
      float accH_xy_sensitivity;
      float accH_xz_sensitivity;
      float accH_yz_sensitivity;
      float accH_roll;
      float accH_pitch;
      float accH_yaw;
      float accH_scale_misalignment[CALIBRATION_MISALIGNMENT_ARRAY_SIZE];

      int16_t gyro_x_bias[CALIBRATION_BIAS_ARRAY_SIZE]; //temperature dependent bias -10C to 50C in 1 deg increments
      int16_t gyro_y_bias[CALIBRATION_BIAS_ARRAY_SIZE];
      int16_t gyro_z_bias[CALIBRATION_BIAS_ARRAY_SIZE];
      uint8_t pad2;
      float gyro_scale[3];
      float gyro_scale_temp[3];
      float gyro_xy_sensitivity;
      float gyro_xz_sensitivity;
      float gyro_yz_sensitivity;
      float gyro_accel_roll;
      float gyro_accel_pitch;
      float gyro_accel_yaw;
      float gyro_scale_misalignment[CALIBRATION_MISALIGNMENT_ARRAY_SIZE];

      int16_t mag_x_bias[CALIBRATION_BIAS_ARRAY_SIZE]; //temperature dependent bias -10C to 50C in 1 deg increments
      int16_t mag_y_bias[CALIBRATION_BIAS_ARRAY_SIZE];
      int16_t mag_z_bias[CALIBRATION_BIAS_ARRAY_SIZE];
      uint8_t pad3;
      float mag_scale[3];
      float mag_scale_temp[3];
      float mag_xy_sensitivity;
      float mag_xz_sensitivity;
      float mag_yz_sensitivity;
      float mag_accel_roll;
      float mag_accel_pitch;
      float mag_accel_yaw;
      float mag_scale_misalignment[CALIBRATION_MISALIGNMENT_ARRAY_SIZE];
      float magnetic_field_magnitude;
      float magnetic_inclination;

      float temperature_bias;
      float temperature_scale;
    } cal_data_v1;
    uint8_t cal_data[2036];//Note: total size of structure is 2048 bytes
  } data;
} calibration_data_t;

typedef enum
{
  WIRELESS_V2_RADIO_MODE_OFF = 0,
  WIRELESS_V2_RADIO_MODE_MESH_ONLY,
  WIRELESS_V2_RADIO_MODE_STREAM_TX,
  WIRELESS_V2_RADIO_MODE_STREAM_RX,
  WIRELESS_V2_RADIO_MODE_BASIC,
  WIRELESS_V2_RADIO_MODE_LISTEN,
  WIRELESS_V2_RADIO_MODE_CARRIER,
  WIRELESS_V2_RADIO_MODE_SPEW,
  WIRELESS_V2_RADIO_MODE_CARRIER_SWEEP,
  WIRELESS_V2_RADIO_MODE_STREAM_TX_RAPID,
  WIRELESS_V2_RADIO_MODE_ENUM_LAST_ELEMENT
} wireless_v2_radio_mode_t;

/**
 * Config settings for the opal. Used in the INI file and the USB communication protocols to configure opals and the AP.
 */
typedef enum {
  CONFIG_V2_OUTPUT_RATE = 0,
  CONFIG_V2_OUTPUT_RATE_WIRELESS_DIVIDER,

  CONFIG_V2_ENABLE_ACCEL,
  CONFIG_V2_ENABLE_GYRO,
  CONFIG_V2_ENABLE_MAG,
  CONFIG_V2_ENABLE_PRESSURE,

  CONFIG_V2_FILE_FORMAT,
  CONFIG_V2_ENABLE_SI_SD_DATA_LOGGING,
  CONFIG_V2_ENABLE_RAW_SD_DATA_LOGGING,

  CONFIG_V2_WIRELESS_PROTOCOL,
  CONFIG_V2_WIRELESS_CHANNEL,
  CONFIG_V2_WIRELESS_TX_POWER,
  CONFIG_V2_WIRELESS_LATENCY_MS,
  CONFIG_V2_WIRELESS_GROUP_CODE,
  CONFIG_V2_WIRELESS_TX_MASK,
  CONFIG_V2_WIRELESS_TARGET_AP_ID,
  CONFIG_V2_WIRELESS_DEBUG_CONTROL,

  CONFIG_V2_FILTER_MODE,
  CONFIG_V2_FILTER_MAX_LATENCY_MS,
  CONFIG_V2_SD_CARD_BUFFERING,

  CONFIG_V2_BATTERY_CUTOFF,
  CONFIG_V2_LED_MODE,
  CONFIG_V2_BUTTON_MODE,
  CONFIG_V2_EXTERNAL_COMMUNICATIONS_MODE, //Values from external_communications_mode_t
  CONFIG_V2_LCD_DISPLAY_ENABLE,
  CONFIG_V2_LCD_DISPLAY_ANGLE,
  CONFIG_V2_NUMBER_OF_APS,

  CONFIG_V2_DEBUG_LCD_ENABLE,
  CONFIG_V2_ENABLE_USB_CDC_ACM_VCOM,
  CONFIG_V2_ENABLE_AP_DATA_STREAMING,
  CONFIG_V2_UNUSED2, //Note: dont remove these as it breaks parsing of old data files that have subsiquent enumeration values set.
  CONFIG_V2_UNUSED3, //Note: dont remove these as it breaks parsing of old data files that have subsiquent enumeration values set.
  CONFIG_V2_UNUSED4, //Note: dont remove these as it breaks parsing of old data files that have subsiquent enumeration values set.

  CONFIG_V2_DISABLE_CALIBRATION_DATA,

  CONFIG_V2_RAW_RATE_OVERRIDE_ACCEL_MID,
  CONFIG_V2_RAW_RATE_OVERRIDE_ACCEL_HIGH,
  CONFIG_V2_RAW_RATE_OVERRIDE_GYRO,
  CONFIG_V2_RAW_RATE_OVERRIDE_MAG,

  CONFIG_V2_RANGE_OVERRIDE_ACCEL_MID,
  CONFIG_V2_RANGE_OVERRIDE_GYRO,
  CONFIG_V2_RANGE_OVERRIDE_MAG,

  CONFIG_V2_LOGGING_BEHAVIOR,

  CONFIG_V2_ENABLE_RANGING,
  CONFIG_V2_SENSOR_INDEX,
  CONFIG_V2_SENSOR_COUNT,

  CONFIG_V2_MOUNT_MODE,

  CONFIG_V2_WIRELESS_MESH_CHANNEL,
  CONFIG_V2_WIRELESS_DOCKING_BEHAVIOR,

  CONFIG_V2_STANDBY_MODE,

  CONFIG_V2_VALUE_LAST_ELEMENT
} config_value_t;


typedef struct {
  uint64_t event_sync_time;//Wireless sync time in us
  uint64_t stm32_time_us;
  uint32_t selected_button_option;
  uint32_t button_event; //of type button_event_t
  uint32_t device_id;
  char     event_string[24];
} PACK_STRUCT_STRUCT button_event_data_t;

#endif /* V2_PUBLIC_TYPES_H */
