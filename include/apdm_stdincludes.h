#ifndef APDM_STDINCLUDES_H_
#define APDM_STDINCLUDES_H_

#ifdef _WINDOWS
#  include <wtypes.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _MSC_VER
//Compiling in a Microsoft Visual Studio Enviroment
//Visual Studio 2005 does not provide some C99 headers
#  include "apdm_compat/stdint.h"
#  include "apdm_compat/stdbool.h"
#else
#  include <stdint.h>
#  include <stdbool.h>
#  include <sys/time.h>
#endif



#ifdef _WINDOWS
#  ifdef __MINGW__
#    include <mingw/_mingw.h>
#  endif
#    define APDM_EXPORT  __declspec(dllexport)
#    define APDM_IMPORT  __declspec(dllimport)
#else
#  define APDM_EXPORT
#  define APDM_IMPORT
#endif

#ifdef __cplusplus
}
#endif

#endif /*APDM_STDINCLUDES_H_*/
