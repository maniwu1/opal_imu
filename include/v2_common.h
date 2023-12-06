#ifndef V2_COMMON_H_
#define V2_COMMON_H_

#include "v2_common_types.h"
#include "v2_common_misc.h"
#include "v2_common_enums.h"

//#define MONITOR_FIRMWARE //this is only defined here to make code highlighting work well.  It should be turned off.

#if USE_SMART_SOX_BUFFERS
// NOTE: USE_CAN_IDE_EXT enables use of EID instead of / on top of SID
//this implementation of the EID is not tested to work appropriately with CANBUS,
//and may fail hard in a multi-device CAN system, or a non-smartsox/ chibios system
//For debug only!
#define USE_CAN_IDE_EXT                     0
#define SENSOR_RETRY_DELAY                  0 // Delay to try to fix monitor magnetometer issue on cold boot on some hardware.
#define RANGING_ENABLE                      0
#else
#define RANGING_ENABLE                      0
#endif


//settings for smartsox-monitor time synchronizer
//time delta buffer length must be a power of 2 for fancy math tricks
#define EXTIO_TIME_DELTA_BUFFER_CONSTANT    4
#define EXTIO_TIME_DELTA_BUFFER_LENGTH      (1<<(EXTIO_TIME_DELTA_BUFFER_CONSTANT))


#define SAMPLE_DATA_BLOCK_SIZE              512


#define APDM_V2_RADIO_MINIMUM_CHANNEL        2
#define APDM_V2_RADIO_MAXIMUM_CHANNEL        81
#define APDM_V2_RADIO_CHANNEL_GAP            7


#define CAN_SID_SEQUENCE_MASK             0x1F


#define DEFAULT_MAX_FILTER_LATENCY_MS    250
#define INFINANT_MAX_LATENCY             UINT16_MAX
#define INFINANT_MAX_LATENCY_2           0


#define FLASH_DEVICE_INFO_FEATURES_MASK_DECAWAVE_PRESENT   (1<<0)

typedef struct {
  uint32_t length;
  uint32_t structure_version;
  union {
    struct {
      uint16_t device_type; //from monitor_product_t typedef
      uint16_t features_bitmap;//Bit 0: decawave present
      uint32_t sensor_id;
      uint16_t hardware_id; //From hardware_id_t typedef enum.
      uint16_t schematic_revision; //A number, programmed by the programming jig. This typically represents minor component value changes for the same PCB hardware_id

      uint32_t led_cal_red;
      uint32_t led_cal_green;
      uint32_t led_cal_blue;
      uint32_t led_cal_temperature_red;
      uint32_t led_cal_temperature_green;
      uint32_t led_cal_temperature_blue;
      char case_id[32];
      char fda_udi[128];//Unique ID for FDA purposes
      char module_id[32];//E.G. SMTC15354190. This is whats on the sticker from STMC

      calibration_data_t cal_data;
      calibration_data_t cal_data_user;
#if USE_SMART_SOX_BUFFERS_CAL
      calibration_data_t cal_data_smart_sox;
#endif
    } PACK_STRUCT_STRUCT  v1;
  } data;
} PACK_STRUCT_STRUCT flash_device_info_t;



#define MAX_SD_CARD_RAW_SENSORS     QUEUE_TYPE_COUNT_END_ELEMENT
#define MAX_SD_CARD_SI_SENSORS      SI_QUEUE_TYPE_LAST_ELEMENT

#define SD_STRUCTURE_MAGIC_NUMBER    'M'

#define USB_STRUCTURE_MAGIC_NUMBER   0xFEDC1234


typedef struct {
  uint16_t payload_size;//includes the payload size itself
  uint8_t structure_type;//Of type sd_structure_type_t
  uint8_t magic_number;
} PACK_STRUCT_STRUCT sd_card_data_header_t;


#define FILE_HEADER_VERSION_STRING_SIZE       8
#define FILE_HEADER_VERSION_STRING            "APDM2001"

typedef struct {
  char file_version_string[FILE_HEADER_VERSION_STRING_SIZE];
  sd_card_data_header_t data_header;

  uint16_t structure_version;
  union {
    struct {
      uint8_t file_format;
      uint8_t num_raw_sensors;
      uint8_t num_si_sensors;
      uint8_t padding;
      uint32_t hardware_id;
      uint32_t monitor_id;
      uint32_t monitor_type;
      uint8_t file_create_year;//Starting at 1900
      uint8_t file_create_month;
      uint8_t file_create_day;
      uint8_t file_create_hour;
      uint8_t file_create_minute;
      uint8_t file_create_second;
      uint64_t microcontroller_time_us;
      uint16_t file_create_milliseconds;
    } PACK_STRUCT_STRUCT v2_0;
    uint8_t bytes[64];
  } PACK_STRUCT_STRUCT data;
} PACK_STRUCT_STRUCT sd_card_file_structure_header_t;


typedef struct {
  sd_card_data_header_t data_header;
  uint32_t byte_offset;//This is the byte offset into the structure that the first data byte starts at
  uint8_t data[128];
} PACK_STRUCT_STRUCT sd_card_cal_data_t;


typedef struct {
  sd_card_data_header_t data_header;
  uint64_t sync_epoch_ms_offset; //Number of ms since Jan 1, 1970
  uint64_t wireless_sync_time_us;
  uint64_t microcontroller_time_ms; //The MCU time when the record was written to the .APDM file.
  uint32_t event_type;//of type monitor_events_t
  uint32_t event_count_or_state;
} PACK_STRUCT_STRUCT sd_card_error_event_count_t;

typedef  sd_card_error_event_count_t sd_card_error_state_t;


typedef struct {
  sd_card_data_header_t data_header;
  uint32_t config_field;
  uint32_t config_value;
} PACK_STRUCT_STRUCT sd_card_config_setting_t;



typedef struct {
  sd_card_data_header_t data_header;
  uint32_t field_type; //of type sd_card_string_value_type_t
  uint32_t string_config; //of type config_v2_string_t
  char the_string[128];
} PACK_STRUCT_STRUCT sd_card_string_value_t;


#define NUM_METRIC_VALUES_PER_STRUCTURE    8
typedef struct {
  sd_card_data_header_t data_header;
  uint32_t metric_type; //of type sd_card_metric_type_t
  uint64_t stm32_time_us;
  int32_t metric_values[NUM_METRIC_VALUES_PER_STRUCTURE];
} PACK_STRUCT_STRUCT sd_card_metrics_data_t;


typedef struct {
  sd_card_data_header_t data_header;
  uint32_t button_event_type; //of type button_event_t
  uint64_t stm32_time_us;
  uint64_t sync_time_us;
  uint32_t selected_button_option;
} PACK_STRUCT_STRUCT sd_card_button_press_data_t;


typedef struct {
  sd_card_data_header_t data_header;
  uint32_t last_button_press_offset;
} PACK_STRUCT_STRUCT sd_card_last_button_press_offset;


typedef struct {
  //Note: this structure is exactly 10 bytes long
  uint8_t sensor_reading_type; //of type sensor_reading_type_t
  uint8_t raw_sensor_temperature_aux;
  int16_t raw_sensor_temperature;

  union {
    struct {
      int16_t raw_v;
      int16_t aux_data_1;
      int16_t aux_data_2;
    } PACK_STRUCT_STRUCT r1;
#if 0
    struct {
      uint16_t raw_v;
      uint16_t aux_data_1;
      uint16_t aux_data_2;
    } PACK_STRUCT_STRUCT ru1;
#endif
    struct {
      int16_t raw_x;
      int16_t raw_y;
      int16_t raw_z;
    } PACK_STRUCT_STRUCT r3;
#if 0
    struct {
      uint16_t raw_x;
      uint16_t raw_y;
      uint16_t raw_z;
    } PACK_STRUCT_STRUCT ru3;
#endif
  } PACK_STRUCT_STRUCT raw;
} PACK_STRUCT_STRUCT raw_generic_sample_t;


#define APDM_TEMPERATURE_SCALING_FACTOR    2
typedef struct {
  //Note: this structure is exactly 16 bytes long
  uint8_t sensor_reading_type; //of type sensor_reading_type_t
  int8_t temperature_c;//Units of 0.5 deg C per LSB, APDM_TEMPERATURE_SCALING_FACTOR
  uint8_t padding1;
  uint8_t padding2;
  union {
    struct {//single values
      float si_v;
    } PACK_STRUCT_STRUCT v1;
    struct {//tripple values
      float si_x;
      float si_y;
      float si_z;
    } PACK_STRUCT_STRUCT v3;
    struct {
      uint64_t anchor_timestamp;
      uint32_t remote_device_id;
    } PACK_STRUCT_STRUCT range;
  } PACK_STRUCT_STRUCT si;
} PACK_STRUCT_STRUCT si_generic_sample_t;

typedef struct {
  uint64_t sync_time_us;//Wireless sync time.
  uint64_t microcontroller_time_us;
  si_generic_sample_t data;
} range_sample_t;



typedef struct {
  uint8_t sensor_reading_type; //of type sensor_reading_type_t
  int8_t temperature_c;//0.5 deg C per LSB (divide by 4 to get actual Deg C)
  union {
    struct {
      uint16_t v1;
    } PACK_STRUCT_STRUCT v1;
    struct {
	  int16_t v1;
	} PACK_STRUCT_STRUCT v1u;
    struct {
      int16_t si_x;
      int16_t si_y;
      int16_t si_z;
    } PACK_STRUCT_STRUCT v3;
  } PACK_STRUCT_STRUCT si;
} PACK_STRUCT_STRUCT si_generic_fixed_point_sample_t;

#define NUM_WIRELESS_PACKET_RANGE_DATA_ELEMENTS          5

#define  FILTERED_DATA_SAMPLE_FLAGS_RAW_DATA_ONLY       (1<<0)
#define  FILTERED_DATA_SAMPLE_FLAGS_RANGE_DATA          (1<<1)


typedef struct {
  sd_card_data_header_t data_header;
  struct {
    uint32_t device_id;
    uint64_t sync_epoch_ms_offset; //Number of ms since Jan 1, 1970
    uint64_t sync_time_us;
    uint64_t microcontroller_time_us;

    uint32_t flags;
  } PACK_STRUCT_STRUCT header;

  raw_generic_sample_t raw_samples[MAX_SD_CARD_RAW_SENSORS];
  si_generic_fixed_point_sample_t si_samples_fixed[MAX_SD_CARD_SI_SENSORS];

} PACK_STRUCT_STRUCT sd_card_data_sample_t;



typedef struct {
  uint32_t remote_device_id;
  uint64_t wireless_sync_time_us;
  uint64_t stm32_time_us;
  uint64_t anchor_timestamp;
} PACK_STRUCT_STRUCT sd_range_data_t;

typedef struct {
  sd_card_data_header_t data_header;
  uint32_t device_id;
  sd_range_data_t ranging_data_value;
} PACK_STRUCT_STRUCT sd_card_range_data_t;


typedef union {
  sd_card_data_sample_t data_sample;
  sd_card_range_data_t range_data;
} sd_card_large_data_union_t;


typedef struct {
  sd_card_data_header_t data_header;
  struct {
    uint32_t device_id;
    uint64_t sync_epoch_ms_offset; //Number of ms since Jan 1, 1970
    uint64_t sync_time_us;
    uint64_t microcontroller_time_us;

    uint32_t flags;
  } PACK_STRUCT_STRUCT header;

  raw_generic_sample_t raw_sample;
}PACK_STRUCT_STRUCT sd_card_raw_data_sample_t;

typedef struct {
  sd_card_data_header_t data_header;
  struct {
    uint32_t device_id;
    uint64_t sync_epoch_ms_offset; //Number of ms since Jan 1, 1970
    uint64_t sync_time_us;
    uint64_t microcontroller_time_us;

    uint32_t flags;
  } PACK_STRUCT_STRUCT header;

  si_generic_fixed_point_sample_t raw_sample_fixed;
} PACK_STRUCT_STRUCT sd_card_si_data_sample_t;




typedef struct {
  uint64_t wireless_sync_time_us;
  uint64_t stm32_time_us;
  uint32_t remote_device_id;
  uint64_t anchor_timestamp;
} PACK_STRUCT_STRUCT wireless_packet_range_data_t;

typedef struct {
    uint64_t sync_epoch_ms_offset;//Number of ms since Jan 1, 1970
    uint64_t wireless_sync_time_us;
    uint64_t microcontroller_time_us;
    uint32_t flags;

    union {
      struct {
        raw_generic_sample_t raw_data_array[QUEUE_TYPE_COUNT_END_ELEMENT];
        si_generic_fixed_point_sample_t si_data_fixed_array[OUTPUT_CLASS_LAST_ELEMENT2];
      } PACK_STRUCT_STRUCT samples;
      wireless_packet_range_data_t range_data_value;
    } PACK_STRUCT_STRUCT data;
} PACK_STRUCT_STRUCT filtered_data_sample_t;

typedef struct {
  uint32_t device_id;
  filtered_data_sample_t  data_sample;
} PACK_STRUCT_STRUCT host_stream_data_sample_t;


typedef struct {
  uint32_t device_id;
  int32_t rx_rssi_value;
  uint32_t wireless_percent;
  uint32_t battery_millivolts;
  uint32_t battery_percent;
  uint32_t battery_time_to_empty_minutes;
} PACK_STRUCT_STRUCT ap_sensor_status_data_t;

typedef struct {
	uint32_t event_type;
	uint32_t event_count;
} PACK_STRUCT_STRUCT ap_event_count_data_t;

typedef struct {
	uint32_t error_state_type;
	uint32_t error_state_status;
} PACK_STRUCT_STRUCT ap_error_state_data_t;


typedef struct {
  union {
    struct {
      uint16_t field; //of type can_set_cmd_t
      uint16_t new_value;
      uint16_t reserved;
      uint16_t result;
    }PACK_STRUCT_STRUCT set_cmd;
    struct {
      uint16_t field; //of type can_query_cmd_t
      uint16_t reserved;
      uint32_t result;
    }PACK_STRUCT_STRUCT query_cmd;
    struct {
      uint64_t magic_number_to_echo;
    }PACK_STRUCT_STRUCT ping;
    struct {
      uint16_t which_pin; //of type can_sync_box_pin_t
      uint32_t time;
      uint16_t field;//Of type can_edge_t
    }PACK_STRUCT_STRUCT edge_detect;
    struct {
      uint16_t status_code; //of type can_packet_status_t
      uint16_t data1;
      uint16_t data2;
      uint16_t data3;
    } PACK_STRUCT_STRUCT error_response;

    struct {
      uint32_t stm32_time_ms;
      uint32_t padding;
    } PACK_STRUCT_STRUCT heartbeat_response;
    uint64_t buff;
  }PACK_STRUCT_STRUCT data;
} PACK_STRUCT_STRUCT can_packet_t;

typedef struct {
  uint8_t data_type;//Of type can_cmd_types_t
  uint8_t padding1;
  uint8_t padding2;
  uint8_t padding3;
  uint64_t wireless_sync_time_us;
  uint64_t ap_stm32_time_us;
  union {
    can_packet_t full_packet;
    uint8_t padding_buffer[12];//8 Bytes for max can payload + IDs
  } data;
} sync_box_event_t;


#define USE_FLOATING_POINT_WIRELESS_PACKETS      FALSE


typedef struct {
  uint8_t type;
  uint8_t packet_id;
  uint16_t group_code;
  uint32_t tx_id;
  uint32_t rx_id;
  uint8_t rssi;
} wireless_packet_header_t;


typedef struct {
  uint8_t wireless_sync_time_us_buf[7];
  uint8_t accl_gyro_types; //Upper and lower 4 bits of type sensor_reading_type_t. Accel is the low order 4 bits, gyro is the high-order 4 bits.
  uint8_t stm32_time_us_buf[7];
  uint8_t mag_pressure_types; //Upper and lower 4 bits of type sensor_reading_type_t. Pressure is the low order 4 bits, Mag is the high order 4 bits.

  int16_t accl_x_si_fixed;
  int16_t accl_y_si_fixed;
  int16_t accl_z_si_fixed;

  int16_t gyro_x_si_fixed;
  int16_t gyro_y_si_fixed;
  int16_t gyro_z_si_fixed;

  int16_t mag_x_si_fixed;
  int16_t mag_y_si_fixed;
  int16_t mag_z_si_fixed;

  int16_t pressure_pa_fixed;
  int16_t temperature_c_fixed;

  uint8_t temperature_types; //Upper and lower 4 bits of type sensor_reading_type_t. Temperature is the low order 4 bits. High order 4 bits is unused.
} PACK_STRUCT_STRUCT wireless_packet_element_t;



typedef struct {
  uint32_t monitor_id;
  uint32_t type;
  uint32_t value;
} wireless_packet_data_count_t;

typedef struct {
  uint32_t target_monitor_id;
  uint32_t config_field_type; //of type config_value_t
  uint32_t config_value;
} wireless_packet_config_set_t;

typedef struct {
  uint32_t monitor_id;
  uint32_t config_field_type; //of type config_value_t
  uint32_t config_value;
} wireless_packet_config_value_t;

typedef struct {
  uint32_t monitor_id;
  uint32_t string_type_index;//of type config_v2_string_t
  char string_data[APDM_CONFIG_V2_STRING_MAX_LENGTH];
} wireless_packet_label_string_t;

typedef struct {
  uint32_t target_monitor_id;
  uint32_t action_type;//of type wireless_command_action_type_t
  uint32_t action_value_optional;
} wireless_packet_command_action_t;



typedef struct {
  uint8_t packet_type; //of type wireless_mesh_data_type_t
  union {
    struct {
      uint8_t padding[3];
      uint32_t max_latency_ms;
    } PACK_STRUCT_STRUCT max_latency;
    struct {
      uint8_t padding[3];
      uint64_t min_sync;
    } PACK_STRUCT_STRUCT minimum_sync_value;
    struct {
      uint8_t padding0;
      uint16_t wireless_ms;
      uint32_t epoch_second; //seconds since 1970
      uint32_t wireless_sync_time_seconds; //(wireless sync value, microseconds) / (1000000)
    } PACK_STRUCT_STRUCT time_set;
  } PACK_STRUCT_STRUCT data;
} PACK_STRUCT_STRUCT wireless_mesh_data_t;



#define NUM_WIRELESS_PACKET_PAYLOAD_ELEMENTS             5
#define NUM_WIRELESS_PACKET_DATA_COUNT_ELEMENTS          18
#define NUM_WIRELESS_PACKET_CONFIG_SET_COUNT_ELEMENTS    18
#define NUM_WIRELESS_PACKET_CONFIG_VALUE_COUNT_ELEMENTS  18
#define NUM_WIRELESS_PACKET_STRING_LABELS                4
#define NUM_WIRELESS_PACKET_BUTTON_EVENTS                2


#ifndef NRF_MAX_PACKET_DATA_SIZE
#define NRF_MAX_PACKET_DATA_SIZE2   234
#else
#define NRF_MAX_PACKET_DATA_SIZE2   NRF_MAX_PACKET_DATA_SIZE
#endif


#define COMPRESSED_RECORD_SIZE_OVERHEAD            (8 + 8 + 2 + 1 + 8 + 6)
#define COMPRESSED_RECORD_WIRELESS_BUFFER_SIZE     (NRF_MAX_PACKET_DATA_SIZE2 - COMPRESSED_RECORD_SIZE_OVERHEAD)

#if 0
#define COMPRESSED_RECORD_SD_CARD_BUFFER_SIZE      COMPRESSED_RECORD_WIRELESS_BUFFER_SIZE
#else
//4 bytes for sizeof(sd_card_data_header_t)
#define COMPRESSED_RECORD_SD_CARD_BUFFER_SIZE      (245 - COMPRESSED_RECORD_SIZE_OVERHEAD - 4)
#endif


#if COMPRESSED_RECORD_SD_CARD_BUFFER_SIZE > COMPRESSED_RECORD_WIRELESS_BUFFER_SIZE
#define COMPRESSED_RECORD_ACTUAL_BUFFER_SIZE       COMPRESSED_RECORD_SD_CARD_BUFFER_SIZE
#else
#define COMPRESSED_RECORD_ACTUAL_BUFFER_SIZE       COMPRESSED_RECORD_WIRELESS_BUFFER_SIZE
#endif


typedef struct {
    uint64_t wireless_sync_value_start_us;
    uint64_t stm32_sync_time_start_us;
    int16_t nominal_delta_us; // 7812
    uint8_t number_of_samples;
    uint8_t sensor_type_list[8]; //4 bits per sensor type, tracks up to 16 types total per compressed packet, of types   sensor_reading_type_t
} PACK_STRUCT_STRUCT v2_compressed_records_header_t;

typedef struct {
  v2_compressed_records_header_t header;
  uint8_t data_buffer[COMPRESSED_RECORD_ACTUAL_BUFFER_SIZE];
} PACK_STRUCT_STRUCT v2_compressed_records_t;//TODO rename this to v2_delta_encoded_records_t


typedef struct {
  v2_compressed_records_header_t header;
  uint8_t data_buffer[COMPRESSED_RECORD_WIRELESS_BUFFER_SIZE];
} PACK_STRUCT_STRUCT v2_delta_encoded_records_wireless_t;



typedef struct {
	uint32_t device_id;
	v2_compressed_records_t compressed_data;
} PACK_STRUCT_STRUCT v2_compressed_host_records_t;


typedef struct {
  uint8_t payload_type;//Of type wireless_packet_payload_type_t
  uint8_t num_elements;
  uint8_t padding1;
  uint8_t padding2;
  union {
    v2_delta_encoded_records_wireless_t delta_encoded_record_data_wireless;
    wireless_packet_element_t data[NUM_WIRELESS_PACKET_PAYLOAD_ELEMENTS];
    wireless_packet_range_data_t range_data[NUM_WIRELESS_PACKET_RANGE_DATA_ELEMENTS];
    wireless_packet_data_count_t error_counts[NUM_WIRELESS_PACKET_DATA_COUNT_ELEMENTS];
    wireless_packet_data_count_t error_states[NUM_WIRELESS_PACKET_DATA_COUNT_ELEMENTS];
    wireless_packet_data_count_t statistic_counts[NUM_WIRELESS_PACKET_DATA_COUNT_ELEMENTS];
    wireless_packet_config_set_t config_set[NUM_WIRELESS_PACKET_CONFIG_SET_COUNT_ELEMENTS];
    wireless_packet_config_value_t config_value[NUM_WIRELESS_PACKET_CONFIG_VALUE_COUNT_ELEMENTS];
    wireless_packet_label_string_t label_strings[NUM_WIRELESS_PACKET_STRING_LABELS];
    wireless_packet_command_action_t action;
    button_event_data_t button_event_data[NUM_WIRELESS_PACKET_BUTTON_EVENTS];
  } data;
} PACK_STRUCT_STRUCT wireless_packet_payload_t;


typedef struct {
  volatile uint32_t config_values[CONFIG_V2_VALUE_LAST_ELEMENT];//of type config_value_t
  char labels[CONFIG_V2_STRING_LAST_VALUE][APDM_CONFIG_V2_STRING_MAX_LENGTH];
} monitor_configuration_t;




#define CONFIG_V2_ENUM_PACKING_8BIT_NUM_ELEMENTS    CONFIG_PACKED_8BIT_V2_LAST_8BIT_VALUE
#define CONFIG_V2_ENUM_PACKING_16BIT_NUM_ELEMENTS   (CONFIG_PACKED_16BIT_V2_LAST_16BIT_VALUE - CONFIG_PACKED_8BIT_V2_LAST_8BIT_VALUE)
#define CONFIG_V2_ENUM_PACKING_32BIT_NUM_ELEMENTS   (CONFIG_PACKED_32BIT_V2_LAST_32BIT_VALUE - CONFIG_PACKED_16BIT_V2_LAST_16BIT_VALUE)
/**
 * This structure provides a tighter memory packing for a full set of V2 config settings. This is necessary because the AP has a large array of
 * config structures and they suck up too much memory otherwise.
 */
typedef struct {
  /*The AP ends up trackign a ton of data for each opal, and this allows for significant memory use reduction over storing everything as uint32s*/
  uint32_t values_32bit[CONFIG_V2_ENUM_PACKING_32BIT_NUM_ELEMENTS];
  uint16_t values_16bit[CONFIG_V2_ENUM_PACKING_16BIT_NUM_ELEMENTS];
  uint8_t values_8bit[CONFIG_V2_ENUM_PACKING_8BIT_NUM_ELEMENTS];
} PACK_STRUCT_STRUCT config_v2_packed_t;



//======================================================================================
// Config
config_v2_enum_packing_t config_v2_packing_index(const config_value_t v);
void config_v2_packed_set(config_v2_packed_t *d, const config_value_t v, uint32_t new_value);
uint32_t config_v2_packed_get(const config_v2_packed_t *d, const config_value_t v);
bool config_include_in_ini_file(const config_value_t v);
bool config_include_in_hdf_file(const config_value_t v);
bool config_value_hex_format(const config_value_t v);


//======================================================================================
// Enum to Enum mappings
queue_type_t apdm_sensor_reading_type_to_to_queue_type_t(const sensor_reading_type_t v);
sensor_reading_type_t apdm_type_class_to_reading_type(const sensor_type_t t, const si_output_sensor_class_t sensor_class);


//======================================================================================
// Helper Functions
uint32_t apdm_calculate_common_header_structure_sum(void);

//======================================================================================
// Wireless related functions
uint64_t wireless_buffer_to_uint64_t(const uint8_t *dest_buff, const uint8_t buff_length);
uint64_t wireless_packet_element_t_get_wireless_sync_time(const wireless_packet_element_t *p);
uint64_t wireless_packet_element_t_get_stm32_time(const wireless_packet_element_t *p);
void wireless_uint64_t_to_buffer(uint64_t value, uint8_t *dest_buff, const uint8_t buff_length);
void wireless_packet_to_host_data_sample(const wireless_packet_element_t *src_data, const wireless_packet_header_t *src_header, host_stream_data_sample_t *dest);
void filtered_data_sample_to_wireless_packet(const filtered_data_sample_t *src_data, wireless_packet_element_t *dest);
sensor_reading_type_t wireless_get_reading_type(const wireless_packet_element_t *src_data, const si_output_sensor_class_t oc);

//======================================================================================
// Data Related Functions
float apdm_sensor_reading_type_max_value(const sensor_reading_type_t v);
float apdm_sensor_reading_type_divisor_factor(const sensor_reading_type_t v);
float apdm_sensor_reading_type_offset(const sensor_reading_type_t v);
bool apdm_check_finite_data(const si_generic_sample_t *data);
bool apdm_si_fixed_to_float(const si_generic_fixed_point_sample_t *src, si_generic_sample_t *dest);
bool apdm_si_float_to_fixed(const si_generic_sample_t *src, si_generic_fixed_point_sample_t *dest);
bool apdm_si_output_sensor_class_t_3d(const si_output_sensor_class_t oc);
bool apdm_is_queue_type_t_3d(const queue_type_t q_type);
bool apdm_is_sensor_reading_type_t_3d(const sensor_reading_type_t v);




#endif /* V2_COMMON_H_ */
