#include "MatrixVoiceFacade.hpp"

MatrixVoiceFacade::MatrixVoiceFacade() {
    // Setup global semaphore for everloop and mics tasks' locking
    initTaskLocker();

    // Init key handlers
    matrixVoiceHandler = new MatrixVoiceHandler();
    networkHandler = new NetworkHandler(matrixVoiceHandler);

    // Everloop task must be setup when network instance is ready, but before actual network setup
    matrixVoiceHandler->initEverloopTask([this]() { this->everloopTaskCallback(); });
    // Setup WiFi and MQTT connections
    networkHandler->setup();
    // Streaming task must be setup last to avoid clashes.
    matrixVoiceHandler->initAudioStreamingTask([this]() { this->audioStreamingTaskCallback(); });
}

/**
 * Main audio task, which streams data captured from microphones via WS client to Vosk server.
 */
[[noreturn]]
void MatrixVoiceFacade::audioStreamingTaskCallback() {
    while (true) {
        // Check if streaming bits are set
        matrixVoiceHandler->waitForAudioStream();
        // We start streaming only when WS client is ready and corresponding task is locked
        if (matrixVoiceHandler->shouldSendAudio() && networkHandler->isSocketConnected() &&
            isTaskAquired(MatrixVoiceHandler::AUDIO_STREAM_BLOCK_TIME)) {
            // Call audio processing API with WS streaming callback
            matrixVoiceHandler->readAudioData([this](uint8_t *voiceBuffer, size_t bufferSize) {
                this->networkHandler->publishData(voiceBuffer, bufferSize);
            });
            releaseTask();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Main LED ring task, which changes colors depending on different events.
 */
[[noreturn]]
void MatrixVoiceFacade::everloopTaskCallback() {
    while (true) {
        // Check if everloop bits are set
        matrixVoiceHandler->waitForEverloop();
        // We change LED colors only when corresponding task is locked
        if (isTaskAquired(MatrixVoiceHandler::EVERLOOP_BLOCK_TIME)) {
            // Call everloop processing API
            matrixVoiceHandler->renderEverloop(
                    networkHandler->isWiFiClientConnected() && networkHandler->isSocketConnected()
            );
            matrixVoiceHandler->releaseEverloop();
            releaseTask();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Keep socket client alive in a loop.
 * Track OTA updates.
 */
void MatrixVoiceFacade::keepAlive() {
    NetworkHandler::trackOTAUpdates();

    if (!networkHandler->isOTAUpdateInProgress()) {
        networkHandler->keepSocketClientAlive();
    }
}
