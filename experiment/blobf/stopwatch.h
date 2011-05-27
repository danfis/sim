#ifndef __STOPWATCH_H__
#define __STOPWATCH_H__

#include <time.h>

struct _stopwatch_t {
    struct timespec start;
    struct timespec stop;
};
typedef struct _stopwatch_t stopwatch_t;

static inline void stopwatchStart(stopwatch_t *t);
static inline void stopwatchStop(stopwatch_t *t);

/**
 * Returns elapsed time via timespec.
 */
static inline void stopwatchElapsed(const stopwatch_t *t, struct timespec *diff);

/**
 * Returns elapsed time in usec.
 */
static inline long stopwatchElapsedUs(const stopwatch_t *t);
static inline long stopwatchElapsedMs(const stopwatch_t *t);


/**** INLINE ****/
static inline void stopwatchStart(stopwatch_t *t)
{
    clock_gettime(CLOCK_MONOTONIC, &t->start);
}
static inline void stopwatchStop(stopwatch_t *t)
{
    clock_gettime(CLOCK_MONOTONIC, &t->stop);
}

static inline void stopwatchElapsed(const stopwatch_t *t, struct timespec *diff)
{
    if (t->stop.tv_nsec > t->start.tv_nsec){
        diff->tv_nsec = t->stop.tv_nsec - t->start.tv_nsec;
        diff->tv_sec = t->stop.tv_sec - t->start.tv_sec;
    }else{
        diff->tv_nsec = t->stop.tv_nsec + 1000000000L - t->start.tv_nsec;
        diff->tv_sec = t->stop.tv_sec - 1 - t->start.tv_sec;
    }
}

static inline long stopwatchElapsedUs(const stopwatch_t *t)
{
    struct timespec diff;
    stopwatchElapsed(t, &diff);
    return diff.tv_nsec / 1000L + diff.tv_sec * 1000000L;
}

static inline long stopwatchElapsedMs(const stopwatch_t *t)
{
    struct timespec diff;
    stopwatchElapsed(t, &diff);
    return diff.tv_nsec / 1000000L + diff.tv_sec * 1000L;
}

#endif /* __STOPWATCH_H__ */
