#ifndef ENCODER_H
#define ENCODER_H
#include "types.h"

int   encoder_init(Encoder* e, const char* bus, float scale, float offset,
                   int inv, pthread_mutex_t* mtx);
float encoder_update(Encoder* e, pthread_mutex_t* mtx);  // poll + filter
float encoder_get(Encoder* e);                           // lock-free cached read
void  encoder_reset_zero(Encoder* e, pthread_mutex_t* mtx);

#endif

void* thread_encoder(void* arg);  // adaptive-rate polling thread (both axes)
