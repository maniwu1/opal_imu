#ifndef DATA_APP_H_
#define DATA_APP_H_

#include "apdm_stdincludes.h"
#include "apdm.h"
#include "apdm_internal_includes.h"
#include "apdm_internal.h"
#include "apdm_calibration.h"
#include "apdm_l2_device.h" //for apdm_copy_calibration_response
#include "tekhex.h"

#include "data_app_hdf.h"


int apdm_get_apdm_file_version_fh(FILE * file_handle, apdm_file_version_t *version);

int apdm_read_file_info(FILE *in_file, apdm_recording_info_t *info);

int apdm_get_hdf_dataset_shape_internal(char *file, int device_id, char *datasetName, int *shape, int *ndims);

int apdm_find_first_and_last(FILE *fin[], const int nFiles, uint64_t *first_sample, uint64_t *last_sample, const int use_sync_lock);

herr_t apdm_close_datasets(hid_t loc_id, const char *name, const H5L_info_t *info, void *opdata);
int apdm_get_hdf_module_list(char *file, uint32_t *module_list, int *nMonitors);


#endif //DATA_APP_H_
