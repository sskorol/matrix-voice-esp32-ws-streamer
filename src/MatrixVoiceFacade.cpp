#include "MatrixVoiceFacade.hpp"

MatrixVoiceFacade::MatrixVoiceFacade() {
  // Setup global semaphore for everloop and mics tasks' locking
  initTaskLocker();
  
  // Init key handlers
  matrixVoiceHandler = new MatrixVoiceHandler();
  networkHandler = new NetworkHandler(matrixVoiceHandler);

  // Everloop task must be setup when network instance is ready, but before actual network setup
  matrixVoiceHandler->initEverloopTask([this]() { return this->everloopTaskCallback(); });
  // Setup WiFi and MQTT connections
  networkHandler->setup();
  // Streaming task must be setup last to avoid clashes.
  matrixVoiceHandler->initAudioStreamingTask([this]() { return this->audioStreamingTaskCallback(); });
}

/**
 * Main audio task, which streams data captured from microphones via sync MQTT client to Kaldi server.
 */
void MatrixVoiceFacade::audioStreamingTaskCallback() {
  while (true) {
    // Check if streaming bits are set
    matrixVoiceHandler->waitForAudioStream();
    // We start streaming only when MQTT client is ready and corresponding task is locked
    if (matrixVoiceHandler->shouldSendAudio() && networkHandler->isSyncMqttClientConnected() && isTaskAquired(MatrixVoiceHandler::AUDIO_STREAM_BLOCK_TIME)) {
      // Call audio processing API with MQTT streaming callback
      matrixVoiceHandler->readAudioData([this](uint8_t *voicebuffer, unsigned int bufferSize) {
        this->networkHandler->publishSync(NetworkHandler::VOICE_STREAM_TOPIC.c_str(), voicebuffer, bufferSize);
      });
      releaseTask();
    }
    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

/**
 * Main LED ring task, which changes colors depending on different events.
 */
void MatrixVoiceFacade::everloopTaskCallback() {
  while (true) {
    // Check if everloop bits are set
    matrixVoiceHandler->waitForEverloop();
    // We change LED colors only when corresponding task is locked
    if (isTaskAquired(MatrixVoiceHandler::EVERLOOP_BLOCK_TIME)) {
      // Call everloop processing API
      matrixVoiceHandler->renderEverloop(networkHandler->isWiFiClientConnected());
      releaseTask();
      matrixVoiceHandler->releaseEverloop();
      Serial.println("[ESP] Updated everloop");
    }
    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

/**
 * Track OTA requests
 */
void MatrixVoiceFacade::checkForUpdates() {
  networkHandler->trackOTAUpdates();
}

/**
 * Keeps MQTT client alive in a loop
 */
void MatrixVoiceFacade::keepMqttAlive() {
  if (!networkHandler->isOTAUpdateInProgress()) {
    networkHandler->keepSyncMqttClientAlive();
  }
}
