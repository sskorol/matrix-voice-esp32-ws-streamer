#include "TasksHandler.hpp"

// Semaphore is required for blocking different tasks like everoop or mics to avoid ESP32 crashing
SemaphoreHandle_t taskLocker;

const int AUDIO_STREAMING_BIT = BIT3;
const int AUDIO_STREAMING_TASK_STACK_SIZE = 10000;
EventGroupHandle_t audioStreamingGroup;
TaskHandle_t audioStreamingTaskHandler;

const int EVERLOOP_BIT = BIT0;
const int EVERLOOP_TASK_STACK_SIZE = 4096;
EventGroupHandle_t everloopGroup;
TaskHandle_t everloopTaskHandler;

// Everloop
void initEverloopTask(TaskFunction_t everloopTask) {
  everloopGroup = xEventGroupCreate();
  xTaskCreatePinnedToCore(everloopTask, "everloopTask", EVERLOOP_TASK_STACK_SIZE, NULL, 5, &everloopTaskHandler, 1);
  xEventGroupSetBits(everloopGroup, EVERLOOP_BIT);
}

void aquireEverloop() {
  xEventGroupSetBits(everloopGroup, EVERLOOP_BIT);
}

void releaseEverloop() {
  xEventGroupClearBits(everloopGroup, EVERLOOP_BIT);
}

void waitForEverloop() {
  xEventGroupWaitBits(everloopGroup, EVERLOOP_BIT, false, false, portMAX_DELAY);
}

// Audio
void initAudioStreamingTask(TaskFunction_t audioStreamingTask) {
  audioStreamingGroup = xEventGroupCreate();
  xTaskCreatePinnedToCore(audioStreamingTask, "microphonesTask", AUDIO_STREAMING_TASK_STACK_SIZE, NULL, 3, &audioStreamingTaskHandler, 0);
  xEventGroupSetBits(audioStreamingGroup, AUDIO_STREAMING_BIT);
}

void aquireAudioStream() {
  xEventGroupSetBits(audioStreamingGroup, AUDIO_STREAMING_BIT);
}

void releaseAudioStream() {
  xEventGroupClearBits(audioStreamingGroup, AUDIO_STREAMING_BIT);
}

void waitForAudioStream() {
  xEventGroupWaitBits(audioStreamingGroup, AUDIO_STREAMING_BIT, false, false, portMAX_DELAY);
}

// Semaphore
void initTaskLocker() {
  if (taskLocker == NULL) {
    taskLocker = xSemaphoreCreateMutex();
    if (taskLocker != NULL) {
      xSemaphoreGive(taskLocker);
    }
  }
}

bool isTaskAquired(unsigned long timeout) {
  return (xSemaphoreTake(taskLocker, (TickType_t)timeout) == pdTRUE);
}

void releaseTask() {
  xSemaphoreGive(taskLocker);
}
