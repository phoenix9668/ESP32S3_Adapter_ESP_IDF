#ifndef GNSS_ML307C_H
#define GNSS_ML307C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t generation;
  bool valid;
  uint8_t fix_type;
  double latitude;
  double longitude;
  double altitude_m;
  double speed_kph;
  double course_deg;
  double hdop;
  uint8_t satellites;
  char utc_time[25];
} gnss_fix_t;

bool gnss_ml307c_parse_fields(const char *const *fields, size_t count,
                              gnss_fix_t *fix);
bool gnss_ml307c_parse_line(const char *line, gnss_fix_t *fix);

#ifdef __cplusplus
}
#endif

#endif
