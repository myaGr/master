#ifndef _GNSS_COMMON_H_
#define _GNSS_COMMON_H_

#include "gnss_types.h"
#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t crc24q(const unsigned char *buff, int32_t len);
extern gtime_t utc2gpst(gtime_t t);
extern gtime_t gpst2utc(gtime_t t);

#ifdef __cplusplus
}
#endif

#endif

