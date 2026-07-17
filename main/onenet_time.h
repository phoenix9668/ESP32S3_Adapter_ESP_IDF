#ifndef ONENET_TIME_H
#define ONENET_TIME_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool onenet_time_parse_cclk(const char *value, uint64_t *utc_epoch);

#ifdef __cplusplus
}
#endif

#endif
