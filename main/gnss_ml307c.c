#include "gnss_ml307c.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GNSS_FIELD_COUNT 12U
#define GNSS_LINE_MAX_LEN 256U

static bool parse_double_value(const char *text, double *value) {
  if (text == NULL || value == NULL || *text == '\0') {
    return false;
  }
  errno = 0;
  char *end = NULL;
  const double parsed = strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

static bool parse_unsigned_value(const char *text, unsigned long *value) {
  if (text == NULL || value == NULL || *text == '\0') {
    return false;
  }
  errno = 0;
  char *end = NULL;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

static bool parse_coordinate(const char *text, bool latitude,
                             double *coordinate) {
  if (text == NULL || coordinate == NULL) {
    return false;
  }
  const size_t length = strlen(text);
  const size_t degree_digits = latitude ? 2U : 3U;
  if (length <= degree_digits + 2U || length >= 32U) {
    return false;
  }

  char copy[32];
  memcpy(copy, text, length + 1U);
  char hemisphere = (char)toupper((unsigned char)copy[length - 1U]);
  if (hemisphere != 'N' && hemisphere != 'S' && hemisphere != 'E' &&
      hemisphere != 'W') {
    return false;
  }
  copy[length - 1U] = '\0';
  if ((latitude && hemisphere != 'N' && hemisphere != 'S') ||
      (!latitude && hemisphere != 'E' && hemisphere != 'W')) {
    return false;
  }

  double degrees_minutes = 0.0;
  if (!parse_double_value(copy, &degrees_minutes)) {
    return false;
  }
  const double degrees = floor(degrees_minutes / 100.0);
  const double minutes = degrees_minutes - degrees * 100.0;
  const double maximum = latitude ? 90.0 : 180.0;
  if (minutes < 0.0 || minutes >= 60.0 || degrees > maximum ||
      (degrees == maximum && minutes > 0.0)) {
    return false;
  }
  double result = degrees + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') {
    result = -result;
  }
  *coordinate = result;
  return true;
}

static void build_utc_time(const char *utc, const char *date, char *output,
                           size_t output_size) {
  output[0] = '\0';
  if (utc == NULL || date == NULL || strlen(utc) < 6U ||
      strlen(date) != 6U) {
    return;
  }
  for (size_t i = 0; i < 6U; ++i) {
    if (!isdigit((unsigned char)utc[i]) || !isdigit((unsigned char)date[i])) {
      return;
    }
  }
  snprintf(output, output_size, "20%c%c-%c%c-%c%cT%c%c:%c%c:%c%cZ",
           date[4], date[5], date[2], date[3], date[0], date[1], utc[0],
           utc[1], utc[2], utc[3], utc[4], utc[5]);
}

bool gnss_ml307c_parse_fields(const char *const *fields, size_t count,
                              gnss_fix_t *fix) {
  if (fields == NULL || fix == NULL || count < GNSS_FIELD_COUNT) {
    return false;
  }

  memset(fix, 0, sizeof(*fix));
  unsigned long fix_type = 0U;
  if (!parse_unsigned_value(fields[5], &fix_type) || fix_type > UINT8_MAX) {
    return false;
  }
  fix->fix_type = (uint8_t)fix_type;
  fix->valid = fix->fix_type == 2U || fix->fix_type == 3U;

  unsigned long satellites = 0U;
  if (parse_unsigned_value(fields[10], &satellites) &&
      satellites <= UINT8_MAX) {
    fix->satellites = (uint8_t)satellites;
  }
  parse_double_value(fields[3], &fix->hdop);
  parse_double_value(fields[4], &fix->altitude_m);
  parse_double_value(fields[6], &fix->course_deg);
  parse_double_value(fields[7], &fix->speed_kph);
  build_utc_time(fields[0], fields[9], fix->utc_time,
                 sizeof(fix->utc_time));

  const bool coordinates_valid =
      parse_coordinate(fields[1], true, &fix->latitude) &&
      parse_coordinate(fields[2], false, &fix->longitude);
  if (fix->valid && !coordinates_valid) {
    fix->valid = false;
  }
  return true;
}

bool gnss_ml307c_parse_line(const char *line, gnss_fix_t *fix) {
  if (line == NULL || fix == NULL) {
    return false;
  }
  static const char prefix[] = "+MGNSSLOC:";
  if (strncmp(line, prefix, sizeof(prefix) - 1U) != 0) {
    return false;
  }
  line += sizeof(prefix) - 1U;
  while (*line == ' ') {
    ++line;
  }
  if (strlen(line) >= GNSS_LINE_MAX_LEN) {
    return false;
  }

  char copy[GNSS_LINE_MAX_LEN];
  strcpy(copy, line);
  const char *fields[GNSS_FIELD_COUNT];
  size_t count = 0U;
  char *start = copy;
  while (count < GNSS_FIELD_COUNT) {
    fields[count++] = start;
    char *comma = strchr(start, ',');
    if (comma == NULL) {
      break;
    }
    *comma = '\0';
    start = comma + 1U;
  }
  return gnss_ml307c_parse_fields(fields, count, fix);
}
