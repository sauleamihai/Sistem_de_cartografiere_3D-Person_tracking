#define _GNU_SOURCE
#include "rt_utils.h"
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>

double time_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Busy-wait instead of nanosleep: avoids the scheduler round-trip and the
// associated jitter on the high-priority RT threads. Costs CPU, buys
// determinism. Only used where the wait is short and timing matters.
void spin_us(long us) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    long target_ns = us * 1000;
    do {
        clock_gettime(CLOCK_MONOTONIC, &now);
    } while ((now.tv_sec - start.tv_sec) * 1000000000L +
             (now.tv_nsec - start.tv_nsec) < target_ns);
}

void spin_ms(long ms) { spin_us(ms * 1000); }

void pin_thread(int core, int priority, const char* name) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    if (priority > 0) {
        struct sched_param sp = { .sched_priority = priority };
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }
    printf("[INIT] %s -> Core %d (prio %d)\n", name, core, priority);
}
