#include "MatrixVoiceHandler.hpp"

// WakeNet constants
const esp_wn_iface_t *MatrixVoiceHandler::WAKE_NET = &WAKENET_MODEL;
const model_coeff_getter_t *MatrixVoiceHandler::MODEL_COEFF_GETTER = &WAKENET_COEFF;

// Microphones constants
const unsigned long MatrixVoiceHandler::AUDIO_STREAM_BLOCK_TIME = 5000;
const unsigned long MatrixVoiceHandler::HOTWORD_RESET_TIMEOUT = 8000;
// Output audio quality depends on buffer size. Don't change it unless you know what you do.
const int MatrixVoiceHandler::VOICE_BUFFER_SIZE = 1024;

// Everloop constants
const unsigned long MatrixVoiceHandler::EVERLOOP_BLOCK_TIME = 10000;

/**
 * Gamma correction is used to be able to change brightness, while keeping the colors appear the same.
 * Check: https://learn.adafruit.com/led-tricks-gamma-correction/the-issue
 */
const uint8_t PROGMEM MatrixVoiceHandler::GAMMA8[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
        2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
        4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8,
        8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13,
        14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21,
        21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30,
        31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42,
        43, 44, 45, 46, 47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57,
        58, 59, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72, 73, 74,
        75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89, 90, 92, 93, 95,
        96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119,
        120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144, 146,
        148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175, 177,
        180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
        215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252,
        255};

MatrixVoiceHandler::MatrixVoiceHandler() {
    // WakeNet model loading (Alexa keyword detection)
    model_data = WAKE_NET->create(MODEL_COEFF_GETTER, DET_MODE_90);
    _shouldSendAudio = false;
    _isHotwordDetected = false;
    initHotwordResetTimer();
    initMatrixCore();
}

/**
 * Required for resetting hotword and everloop state if we don't receive a transcribe from Kaldi.
 */
void MatrixVoiceHandler::resetHotword() {
    if (isHotwordDetected()) {
        changeHotwordState(false);
        acquireEverloop();
    }
}

/**
 * Create RTOS timer for resetting a hotword state if we don't receive a transcribe from Kaldi.
 */
void MatrixVoiceHandler::initHotwordResetTimer() {
    hotwordTimer = xTimerCreate("hotwordTimer", pdMS_TO_TICKS(HOTWORD_RESET_TIMEOUT), pdFALSE, this,
                                hotwordCallbackWrapper);
    Serial.println("[ESP] Created hotword timer");
}

/**
 * Start a timer for hotword resetting.
 */
void MatrixVoiceHandler::startHotwordTimer() {
    xTimerStart(hotwordTimer, 0);
}

/**
 * Check if hotword was recognized by WakeNet.
 */
bool MatrixVoiceHandler::isHotwordDetected() const {
    return _isHotwordDetected;
}

/**
 * Update hotword state from outside to either start hotword detection or stream audio to Kaldi.
 */
void MatrixVoiceHandler::changeHotwordState(bool isHotwordDetected) {
    _isHotwordDetected = isHotwordDetected;
}

/**
 * Common Matrix Voice API setup.
 */
void MatrixVoiceHandler::initMatrixCore() {
    bus.Init();
    everloop.Setup(&bus);
    microphoneArray.Setup(&bus);
    microphoneArray.SetSamplingRate(RATE);
    microphoneArray.SetGain(GAIN);
    matrix_hal::MicrophoneCore mic_core(microphoneArray);
    mic_core.Setup(&bus);
    Serial.println("[ESP] Configured Matrix Core");
}

/**
 * Create and acquire Everloop task for managing LEDs state.
 */
void MatrixVoiceHandler::initEverloopTask(TaskCallbackFunc _everloopCallbackFn) {
    everloopCallbackFn = std::move(_everloopCallbackFn);
    everloopGroup = xEventGroupCreate();
    xTaskCreatePinnedToCore(everloopTaskWrapper, "everloopTask", EVERLOOP_TASK_STACK_SIZE, this, 5,
                            &everloopTaskHandler, 1);
    acquireEverloop();
}

/**
 * When we set Everloop bits, facade's callback is called if there are no blocking tasks present.
 */
void MatrixVoiceHandler::acquireEverloop() {
    xEventGroupSetBits(everloopGroup, EVERLOOP_BIT);
}

/**
 * When we release Everloop bits, other tasks may be locked for further processing.
 */
void MatrixVoiceHandler::releaseEverloop() {
    xEventGroupClearBits(everloopGroup, EVERLOOP_BIT);
}

/**
 * Wait until Everloop bits become available for corresponing callback processing.
 */
void MatrixVoiceHandler::waitForEverloop() {
    xEventGroupWaitBits(everloopGroup, EVERLOOP_BIT, false, false, portMAX_DELAY);
}

/**
 * Adjust preceded colors with brightness and gamma correction.
 */
unsigned int *MatrixVoiceHandler::adjustColors(const unsigned int *colors) const {
    auto *rgbwColors = new unsigned int[COLORS_AMOUNT];
    for (unsigned int i = 0; i < COLORS_AMOUNT; i++) {
        const unsigned int color = floor(brightness * colors[i] / 100.0);
        rgbwColors[i] = pgm_read_byte(&GAMMA8[color]);
    }
    return rgbwColors;
}

/**
 * Base rendering method for Everloop LED ring.
 * Current algorithm assumes turning on only those LEDs that have id < renderingLimit parameter.
 */
void MatrixVoiceHandler::changeEverloopColors(const unsigned int *rgbwColors, unsigned int renderingLimit) {
    const unsigned int ledsSize = everloopImage.leds.size();
    const unsigned int *rgbwNoColors = adjustColors(noColors);

    for (int i = 0; i < ledsSize; i++) {
        matrix_hal::LedValue &currentLed = everloopImage.leds[i];
        if (i <= renderingLimit) {
            currentLed.Set(rgbwColors[0], rgbwColors[1], rgbwColors[2], rgbwColors[3]);
        } else {
            currentLed.Set(rgbwNoColors[0], rgbwNoColors[1], rgbwNoColors[2], rgbwNoColors[3]);
        }
        everloop.Write(&everloopImage);
    }

    delete rgbwNoColors;
}

/**
 * This API is iteratively called when OTA update is in progress.
 * When we receive a current progress from OTA callback, we can project it to LEDs.
 */
void MatrixVoiceHandler::renderOTAUpdateProgress(unsigned int estimatedUpdateProgress) {
    const unsigned int ledsSize = everloopImage.leds.size();
    const unsigned int *rgbwColors = adjustColors(updateInProgressColors);
    // Transform current progress from [0:100] to [0:LEDS_AMOUNT] range
    const float progress = std::round(((float) estimatedUpdateProgress / 100.0f) * (float) ledsSize);
    // Go though LEDs to reflect current progress
    changeEverloopColors(rgbwColors, (unsigned int) progress);
    delete rgbwColors;
}

/**
 * Main rendering Everloop API. Reflects connection, hotword and idle states.
 */
void MatrixVoiceHandler::renderEverloop(bool isConnected) {
    unsigned int *rgbwColors;
    const unsigned int ledsSize = everloopImage.leds.size();

    // Adjust colors with brightness and gamma correction depending on state
    if (isHotwordDetected()) {
        rgbwColors = adjustColors(hotwordColors);
    } else if (!isConnected) {
        rgbwColors = adjustColors(noConnectionColors);
    } else {
        rgbwColors = adjustColors(idleColors);
    }

    // Animate LED ring: outer loop controls the number of LEDs we need to turn on / off
    unsigned int ledNumber = 0;
    do {
        changeEverloopColors(rgbwColors, ledNumber);
        delay(EVERLOOP_OTA_UPDATE_ANIMATION_DELAY);
    } while (ledNumber++ < ledsSize);

    delete rgbwColors;
}

/**
 * Main audio straming API for hotword detection and speech recognition.
 */
void MatrixVoiceHandler::readAudioData(const AudioCallbackFunc &audioCallback) {
    // Read Matrix Voice microphones data
    if (!microphoneArray.Read()) {
        Serial.print("[ESP] Unable to read microphones array data.");
        return;
    }

    // Prepare a beamformed voice buffer
    int16_t voicebuffer[VOICE_BUFFER_SIZE];
    for (uint16_t i = 0; i < VOICE_BUFFER_SIZE; i++) {
        voicebuffer[i] = microphoneArray.Beam(i);
    }

    // Listening for a hotword
    if (!isHotwordDetected()) {
        // If detected, animate LEDs and release a flag for further audio streaming to Vosk server
        int resultCode = WAKE_NET->detect(model_data, voicebuffer);
        if (resultCode > 0) {
            Serial.println("[ESP] Detected hotword");
            changeHotwordState(true);
            acquireEverloop();
            startHotwordTimer();
        }
    }

    // When we detected a hotword, we can now call socket callback to stream audio to Vosk server
    if (isHotwordDetected()) {
        // This size is intended. Don't change it unless you know what you do. Other values may damage an output audio.
        audioCallback((uint8_t *) voicebuffer, VOICE_BUFFER_SIZE);
    }
}

/**
 * Update audio flag from outside to block / unblock streaming.
 */
void MatrixVoiceHandler::changeAudioState(bool shouldSendAudio) {
    _shouldSendAudio = shouldSendAudio;
}

/**
 * Change Microphones gain.
 */
void MatrixVoiceHandler::changeAudioGain(uint16_t gain) {
    microphoneArray.SetGain(gain);
}

/**
 * Change Microphones sampling rate.
 */
void MatrixVoiceHandler::changeAudioRate(uint32_t rate) {
    microphoneArray.SetSamplingRate(rate);
}

/**
 * Check if we should start streaming audio. This flag is activated when WS connection is established.
 */
bool MatrixVoiceHandler::shouldSendAudio() const {
    return _shouldSendAudio;
}

/**
 * Setup a primary task for hotword detection and streaming audio via WS to Vosk server.
 * Callback is provided via MatrixVoiceFacade.
 */
void MatrixVoiceHandler::initAudioStreamingTask(TaskCallbackFunc _audioStreamCallbackFn) {
    audioStreamCallbackFn = std::move(_audioStreamCallbackFn);
    audioStreamingGroup = xEventGroupCreate();
    xTaskCreatePinnedToCore(audioStreamingTaskWrapper, "microphonesTask", AUDIO_STREAMING_TASK_STACK_SIZE, this, 3,
                            &audioStreamingTaskHandler, 0);
}

/**
 * When we acquire Microphones bits, we can start sending audio stream for further recognition.
 */
void MatrixVoiceHandler::acquireAudioStream() {
    xEventGroupSetBits(audioStreamingGroup, AUDIO_STREAMING_BIT);
}

/**
 * When we release Mics bits, other tasks may be locked for further processing.
 */
void MatrixVoiceHandler::releaseAudioStream() {
    xEventGroupClearBits(audioStreamingGroup, AUDIO_STREAMING_BIT);
}

/**
 * Wait until Microphones bits are acquired for further audio stream processing.
 */
void MatrixVoiceHandler::waitForAudioStream() {
    xEventGroupWaitBits(audioStreamingGroup, AUDIO_STREAMING_BIT, false, false, portMAX_DELAY);
}
