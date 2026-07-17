#include "onenet_time.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static bool is_leap_year(int year) {
  return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

static int days_in_month(int year, int month) {
  static const uint8_t days[] = {31, 28, 31, 30, 31, 30,
                                 31, 31, 30, 31, 30, 31};
  if (month == 2 && is_leap_year(year)) {
    return 29;
  }
  return month >= 1 && month <= 12 ? days[month - 1] : 0;
}

static int64_t days_from_civil(int year, unsigned month, unsigned day) {
  year -= month <= 2U;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era = (unsigned)(year - era * 400);
  const unsigned adjusted_month = month > 2U ? month - 3U : month + 9U;
  const unsigned day_of_year =
      (153U * adjusted_month + 2U) / 5U + day - 1U;
  const unsigned day_of_era =
      year_of_era * 365U + year_of_era / 4U - year_of_era / 100U +
      day_of_year;
  return (int64_t)era * 146097 + (int64_t)day_of_era - 719468;
}

bool onenet_time_parse_cclk(const char *value, uint64_t *utc_epoch) {
  if (value == NULL || utc_epoch == NULL) {
    return false;
  }
  while (isspace((unsigned char)*value)) {
    ++value;
  }
  if (*value == '"') {
    ++value;
  }

  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  char sign = '\0';
  int quarter_hours = 0;
  const int fields = sscanf(value, "%2d/%2d/%2d,%2d:%2d:%2d%c%2d", &year,
                            &month, &day, &hour, &minute, &second, &sign,
                            &quarter_hours);
  if (fields < 6 || year < 24 || year > 99 || month < 1 || month > 12 ||
      day < 1 || day > days_in_month(2000 + year, month) || hour < 0 ||
      hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) {
    return false;
  }
  /*
   * ML307C reports the RTC value in UTC while retaining the NITZ timezone
   * suffix.  For example, on a China Telecom network the hour field matches
   * UTC and the response still ends in +32.  Treating the field as local time
   * would subtract eight hours twice and immediately expire a one-hour
   * OneNET token.  Validate the suffix, but do not apply it to the RTC value.
   */
  if (fields >= 8) {
    if ((sign != '+' && sign != '-') || quarter_hours < 0 ||
        quarter_hours > 96) {
      return false;
    }
  }

  const int64_t days =
      days_from_civil(2000 + year, (unsigned)month, (unsigned)day);
  int64_t epoch = days * 86400 + hour * 3600 + minute * 60 + second;
  if (epoch < 1704067200LL) {
    return false;
  }
  *utc_epoch = (uint64_t)epoch;
  return true;
}
