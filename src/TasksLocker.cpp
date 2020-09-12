#include "TasksLocker.hpp"

SemaphoreHandle_t taskLocker;

/**
 * Create a new semaphore for tasks locking.
 */
void initTaskLocker() {
    if (taskLocker == nullptr) {
        taskLocker = xSemaphoreCreateMutex();
        if (taskLocker != nullptr) {
            xSemaphoreGive(taskLocker);
        }
    }
}

/**
 * Try to aquire semaphore to lock required task.
 */
bool isTaskAquired(unsigned long timeout) {
    return (xSemaphoreTake(taskLocker, (TickType_t) timeout) == pdTRUE);
}

/**
 * Release semaphore when task is finished.
 */
void releaseTask() {
    xSemaphoreGive(taskLocker);
}
