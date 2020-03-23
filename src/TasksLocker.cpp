#include "TasksLocker.hpp"

/**
 * Semaphore is required for blocking different tasks like everoop or mics to avoid ESP32 crashing.
 */
SemaphoreHandle_t taskLocker;

/**
 * Create a new semaphore for tasks locking.
 */
void initTaskLocker() {
  if (taskLocker == NULL) {
    taskLocker = xSemaphoreCreateMutex();
    if (taskLocker != NULL) {
      xSemaphoreGive(taskLocker);
    }
  }
}

/**
 * Try to aquire semaphore to lock required task.
 */
bool isTaskAquired(unsigned long timeout) {
  return (xSemaphoreTake(taskLocker, (TickType_t)timeout) == pdTRUE);
}

/**
 * Release semaphore when task is finished.
 */
void releaseTask() {
  xSemaphoreGive(taskLocker);
}
