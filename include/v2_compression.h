#ifndef V2_COMPRESSION_H_
#define V2_COMPRESSION_H_

#include "v2_common_types.h"
#include "v2_common.h"

#ifdef CORTEX_USE_FPU
#include "nrf51_protocol.h"
#endif

//The DISABLE_COMPRESSION flag is used to test the performance impacts of zero compression on the queues of the firmware.
#define DISABLE_COMPRESSION              0

#define MAX_DELTA_ENCODED_DATA_SAMPLES   13


#define INT24_MIN		(-8388607-1)
#define INT24_MAX		(8388607)

typedef uint32_t compression_bitmask_datatype_t;//Type of variable used for the bitmask when compressing data

typedef enum {
	V2_DECOMPRESSION_RESPONSE_SUCCESS = 0,
	V2_DECOMPRESSION_RESPONSE_FAILED,
	V2_DECOMPRESSION_RESPONSE_NO_MORE_DATA
} v2_decompression_resposne_t;



typedef struct {
  bool set_sensor_types_flag;
  uint32_t buffer_offset;
  uint32_t max_sample_size_bytes;
  uint32_t max_buffer_size;
  filtered_data_sample_t last_encoded_filtered_sample;
} v2_compressed_record_encode_tracking;

typedef struct {
  uint32_t decoding_bit_index;
  uint32_t sample_decode_number;
  uint32_t decode_buffer_offset;
  filtered_data_sample_t last_decoded_sample;

} v2_compressed_record_decode_tracking;

typedef struct {
  sd_card_data_header_t data_header;
  struct {
    uint32_t device_id;
    uint64_t sync_epoch_ms_offset; //Number of ms since Jan 1, 1970
  } PACK_STRUCT_STRUCT header;

  v2_compressed_records_t compressed_data;
} PACK_STRUCT_STRUCT sd_card_compressed_data_samples_t;


bool v2_compressed_records_t_validate(const v2_compressed_records_t *data, const uint32_t expected_sensor_count);
void v2_compressed_records_t_init(v2_compressed_record_encode_tracking *tracking, v2_compressed_record_decode_tracking *decode_tracking, v2_compressed_records_t *data, const int32_t nominal_delta, const uint32_t max_buffer_length);
uint32_t v2_compressed_get_data_size_bytes(const sensor_reading_type_t t);
void v2_compressed_records_t_append_sensor_reading_type(v2_compressed_record_encode_tracking *tracking, v2_compressed_records_t *data, const sensor_reading_type_t t);
bool v2_compressed_records_t_more_to_decode(const v2_compressed_record_decode_tracking *tracking, const v2_compressed_records_t *data);
v2_decompression_resposne_t v2_compressed_records_t_decode_sample_reading2(v2_compressed_record_decode_tracking *tracking,
                                                                           const v2_compressed_records_t *data,
                                                                           filtered_data_sample_t *filtered_sample);
bool v2_compressed_records_t_decode_sample_reading(v2_compressed_record_decode_tracking *tracking, const v2_compressed_records_t *data, filtered_data_sample_t *filtered_sample);
bool v2_compressed_records_t_append_sample_reading(v2_compressed_record_encode_tracking *tracking, v2_compressed_records_t *data, const filtered_data_sample_t *filtered_sample, const config_v2_si_sd_data_logging_type_t log_data_type);

uint32_t v2_compressed_records_t_get_bytes_remaining_in_buffer(const v2_compressed_record_encode_tracking *tracking);

void v2_delta_encoded_records_wireless_t_to_v2_compressed_records_t(v2_compressed_records_t *dest, const v2_delta_encoded_records_wireless_t *src);
void v2_compressed_records_t_to_v2_delta_encoded_records_wireless_t(v2_delta_encoded_records_wireless_t *dest, const v2_compressed_records_t *src);

const char* v2_decompression_resposne_t_to_str(const v2_decompression_resposne_t v);

#endif /* V2_COMPRESSION_H_ */
