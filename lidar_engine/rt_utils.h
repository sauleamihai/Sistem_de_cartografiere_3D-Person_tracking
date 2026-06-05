#ifndef RT_UTILS_H
#define RT_UTILS_H

// Real-time primitives: monotonic timing, syscall-free busy waits,
// and thread core-pinning + SCHED_FIFO setup.

double time_now(void);          // monotonic seconds
void   spin_us(long us);        // busy wait, no syscall (safe on RT path)
void   spin_ms(long ms);
void   pin_thread(int core, int priority, const char* name);

#endif
