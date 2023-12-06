#ifndef V2_COMMON_LOGGING_H_
#define V2_COMMON_LOGGING_H_



#ifdef CORTEX_USE_FPU
//This block is used when compiling the opal firmware
#include "project_common_all.h"
//Redefine these as nothing when compiling for the opal firmware
#define apdm_log_debug(...)
#define apdm_log_debug2(...)
#define apdm_log_error(...)
#else
#include "apdm_logging.h"
#endif

#endif /* V2_COMMON_LOGGING_H_ */
