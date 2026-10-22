#ifndef REAL_TIME_HANDLER_H
#define REAL_TIME_HANDLER_H

#include <time.h>

typedef struct {
    struct timespec nextWakeUpTime;
    long sampleTime_ns;
} PeriodicTimer;

void initSystemConfigs();
void startTimer(PeriodicTimer *timer, float sampleTime_s);
void waitNextPeriod(PeriodicTimer *timer);

#endif