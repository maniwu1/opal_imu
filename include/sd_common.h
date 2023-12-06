#ifndef SD_COMMON_H_
#define SD_COMMON_H_

#include "v2_common_types.h"
#include "v2_common.h"
#include "v2_compression.h"

#ifdef CORTEX_USE_FPU
#  include "ff.h"
#else
#  define BaseSequentialStream   void
#  define FIL                    FILE
#  define FRESULT                int
#  define FR_OK                  0
#  define error_log_append(x)    apdm_log_error("Error Event: " #x "\n")
typedef unsigned int UINT;
#endif

bool sdcard_next_read_sample(FIL *fh, host_stream_data_sample_t *host_data_out, sd_card_compressed_data_samples_t *dest_compressed_data, sd_card_file_structure_header_t *header_config, sd_card_large_data_union_t *temp_data_ptr, BaseSequentialStream *chp);
void sdcard_sd_sample_to_host(const sd_card_data_sample_t *src, host_stream_data_sample_t *dest, sd_card_file_structure_header_t *header_config);
bool sdcard_read_file_header(FIL *fh, sd_card_file_structure_header_t *header);

#endif /* SD_COMMON_H_ */
