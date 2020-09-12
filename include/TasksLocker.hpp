#ifndef TASKS_LOCKER_HPP
#define TASKS_LOCKER_HPP

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/semphr.h"
}

/**
 * Semaphore is required for blocking different tasks like everoop or mics to avoid ESP32 crashing.
 */
extern SemaphoreHandle_t taskLocker;

// Global API for locking everloop and mics tasks. See MatrixVoiceFacade for details.
void initTaskLocker();

bool isTaskAquired(unsigned long timeout);

void releaseTask();

#endif
