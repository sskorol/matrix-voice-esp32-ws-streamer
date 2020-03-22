#ifndef TASKS_HANDLER_HPP
#define TASKS_HANDLER_HPP

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/event_groups.h"
  #include "freertos/timers.h"
  #include "freertos/semphr.h"
}

// Everloop
void initEverloopTask(TaskFunction_t everloopTask);
void aquireEverloop();
void releaseEverloop();
void waitForEverloop();

// Audio
void initAudioStreamingTask(TaskFunction_t audioStreamingTask);
void aquireAudioStream();
void releaseAudioStream();
void waitForAudioStream();

// Semaphore
void initTaskLocker();
bool isTaskAquired(unsigned long timeout);
void releaseTask();

#endif
