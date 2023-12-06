#ifndef V2_COMMON_MISC_H_
#define V2_COMMON_MISC_H_

#include "v2_common_types.h"

float saturate_f(const float value, const float min_value, const float max_value);
int32_t strcasecmpLocal(const char* s1, const char* s2);
int32_t strncasecmpLocal(const char* s1, const char* s2, size_t n);

#endif /* V2_COMMON_MISC_H_ */
