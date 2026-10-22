#include "realTimeHandler.h"
#include <stdio.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>

void initSystemConfigs() {
    // MCL_CURRENT: 현재 내가 쓰고 있는 변수, 코드, 스택 메모리를 모두 RAM에 고정.
    // MCL_FUTURE: malloc을 쓰든 함수를 호출하여 스택이 늘어나든 앞으로 쓰일 모든 메모리를 RAM에 고정.
    // 즉, 정책을 설정하는 것.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall");
    }

    // main()의 우선순위를 설정 (단일 프로세스)
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 80; // 1~99, needs root
    // 80은 매우 높은 순위.
    // 잘못하면 마우스나 키보드 입력도 받지 못하는 상황이 발생할 수 있음.
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("sched_setscheduler");
    }
}

// 타이머 초기화 및 시작 시간 설정
void startTimer(PeriodicTimer *timer, float sampleTime_s) {
    timer->sampleTime_ns = static_cast<long>(sampleTime_s * 1e9);
    clock_gettime(CLOCK_MONOTONIC, &timer->nextWakeUpTime);
}

// 다음 주기까지 대기하며, 주기를 놓쳤는지 확인 (Overload 처리)
void waitNextPeriod(PeriodicTimer *timer) {
    struct timespec currentTime;
    
    // 다음 주기 계산
    timer->nextWakeUpTime.tv_nsec += timer->sampleTime_ns;
    while (timer->nextWakeUpTime.tv_nsec >= 1000000000L) {
        timer->nextWakeUpTime.tv_nsec -= 1000000000L;
        timer->nextWakeUpTime.tv_sec += 1;
    }

    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    
    // 제어주기 놓쳤는지 확인
    if (currentTime.tv_sec > timer->nextWakeUpTime.tv_sec || 
       (currentTime.tv_sec == timer->nextWakeUpTime.tv_sec && currentTime.tv_nsec > timer->nextWakeUpTime.tv_nsec)) {
        
        printf("[WARNING] System Overloaded! Resetting next wake-up time.\n");

        timer->nextWakeUpTime = currentTime; 
        timer->nextWakeUpTime.tv_nsec += timer->sampleTime_ns;
        
        while (timer->nextWakeUpTime.tv_nsec >= 1000000000L) {
            timer->nextWakeUpTime.tv_nsec -= 1000000000L;
            timer->nextWakeUpTime.tv_sec += 1;
        }
    }

    // 대기
    int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &timer->nextWakeUpTime, NULL);
    if (ret != 0) {
        printf("Interrupt occurred in clock_nanosleep(): %d\n", ret);
        perror("clock_nanosleep");
    }
}