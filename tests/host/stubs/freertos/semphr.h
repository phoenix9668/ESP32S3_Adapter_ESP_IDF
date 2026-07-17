#ifndef SEMPHR_H
#define SEMPHR_H

#include <stdlib.h>

typedef void *SemaphoreHandle_t;

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
  return malloc(1U);
}

static inline int xSemaphoreTake(SemaphoreHandle_t semaphore,
                                 unsigned int timeout) {
  (void)semaphore;
  (void)timeout;
  return 1;
}

static inline int xSemaphoreGive(SemaphoreHandle_t semaphore) {
  (void)semaphore;
  return 1;
}

static inline void vSemaphoreDelete(SemaphoreHandle_t semaphore) {
  free(semaphore);
}

#endif
