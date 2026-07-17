#ifndef ONENET_REPLY_H
#define ONENET_REPLY_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

bool onenet_reply_parse(const char *payload, size_t payload_length, char *id,
                        size_t id_size, int *code);

#ifdef __cplusplus
}
#endif

#endif
