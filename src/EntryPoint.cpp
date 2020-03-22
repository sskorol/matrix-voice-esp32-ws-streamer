#include "NetworkHandler.hpp"

NetworkHandler *networkHandler;
MatrixVoiceHandler *matrixVoiceHandler;

// Primary callbacks which require network and matrix voice instances to proceed
void audioStreamingTaskCallback(void *p) {
  while (true) {
    waitForAudioStream();
    if (matrixVoiceHandler->shouldSendAudio() && networkHandler->isSyncMqttClientConnected() && isTaskAquired(MatrixVoiceHandler::AUDIO_STREAM_BLOCK_TIME)) {
      matrixVoiceHandler->readAudioData([](uint8_t *voicebuffer, unsigned int bufferSize) {
        networkHandler->publishSync(NetworkHandler::AUDIO_FRAME_TOPIC.c_str(), voicebuffer, bufferSize);
      });
      releaseTask();
    }

    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

void everloopTaskCallback(void *p) {
  while (true) {
    waitForEverloop();
    if (isTaskAquired(MatrixVoiceHandler::EVERLOOP_BLOCK_TIME)) {
      matrixVoiceHandler->renderEverloop(networkHandler->isWiFiClientConnected());
      releaseTask();
      releaseEverloop();
      Serial.println("[ESP] Updated everloop");
    }
    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  initTaskLocker();
  matrixVoiceHandler = (new MatrixVoiceHandler())->init();
  networkHandler = new NetworkHandler(matrixVoiceHandler);
  initEverloopTask(everloopTaskCallback);
  networkHandler->init();
  initAudioStreamingTask(audioStreamingTaskCallback);
}

void loop() {
  networkHandler->trackOTAUpdates();
  if (!networkHandler->isOTAUpdateInProgress()) {
    networkHandler->keepSyncMqttClientAlive();
  }
  delay(1);
}
