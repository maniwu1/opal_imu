#ifndef V2_COMMON_TYPES_H_
#define V2_COMMON_TYPES_H_

#ifdef CORTEX_USE_FPU
#  include "monitor_stdinc.h"
#else
#  define   PACK_STRUCT_STRUCT   __attribute__((packed))
#  include <stdint.h>
#  include <stdbool.h>
#  include <string.h>
#  include <ctype.h>
#endif

#include <sys/types.h>

#include "v2_common_defines.h"


#define GCC_NONNULL   __attribute__((nonnull))


#endif /* V2_COMMON_TYPES_H_ */
