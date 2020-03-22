#ifndef TASKS_HANDLER_HPP
#define TASKS_HANDLER_HPP

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/semphr.h"
}

// Global API for locking everloop and mics tasks. See MatrixVoiceFacade for details.
void initTaskLocker();
bool isTaskAquired(unsigned long timeout);
void releaseTask();

#endif
