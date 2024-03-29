#ifndef MATRIX_VOICE_HANDLER_HPP
#define MATRIX_VOICE_HANDLER_HPP

#include <Arduino.h>
#include <cmath>
#include <utility>
#include <thread>
#include <AsyncMqttClient.h>
#include <WebSocketsClient.h>

#include "everloop.h"
#include "everloop_image.h"
#include "microphone_array.h"
#include "microphone_core.h"
#include "voice_memory_map.h"
#include "wishbone_bus.h"
#include "RingBuf.h"

extern "C" {
#include "esp_wn_iface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
}

// Microphone settings
#define RATE 16000
#define GAIN 4

// WakeNet types
#define WAKENET_COEFF get_coeff_wakeNet3_model_float
#define WAKENET_MODEL esp_sr_wakenet3_quantized

extern const esp_wn_iface_t esp_sr_wakenet3_quantized;
extern const model_coeff_getter_t get_coeff_wakeNet3_model_float;

/**
 * Define callback types for everloop and microphones. See implementations in MatrixVoiceFacade.
 */
typedef std::function<void(uint8_t *voiceBuffer, size_t bufferSize)> AudioCallbackFunc;
typedef std::function<void()> TaskCallbackFunc;

/**
 * A core handler for Matrix LEDs and Microphones control.
 */
class MatrixVoiceHandler {
private:
    // Matrix Voice
    matrix_hal::WishboneBus bus;
    matrix_hal::MicrophoneArray microphoneArray;
    matrix_hal::Everloop everloop;
    matrix_hal::EverloopImage everloopImage;

    // WakeNet
    static const esp_wn_iface_t *WAKE_NET;
    static const model_coeff_getter_t *MODEL_COEFF_GETTER;
    model_iface_data_t *model_data;

    // If there's an issue receiving a voice transcription, this timer will reset hotword state
    TimerHandle_t hotwordTimer{};
    static const unsigned long HOTWORD_RESET_TIMEOUT;

    // Microphones
    const int AUDIO_STREAMING_BIT = BIT3;
    const int AUDIO_STREAMING_TASK_STACK_SIZE = 10000;
    EventGroupHandle_t audioStreamingGroup{};
    TaskHandle_t audioStreamingTaskHandler{};

    static const int VOICE_BUFFER_SIZE;
    bool _isHotwordDetected;
    bool _shouldSendAudio;

    // Everloop
    const int EVERLOOP_BIT = BIT0;
    const int EVERLOOP_TASK_STACK_SIZE = 4096;
    const long EVERLOOP_OTA_UPDATE_ANIMATION_DELAY = 15;
    EventGroupHandle_t everloopGroup{};
    TaskHandle_t everloopTaskHandler{};

    // LED colors settings for different events
    static const uint8_t PROGMEM GAMMA8[];
    static const unsigned int COLORS_AMOUNT = 4;
    const unsigned int brightness = 15;

    unsigned int hotwordColors[COLORS_AMOUNT] = {0, 255, 0, 0};
    unsigned int idleColors[COLORS_AMOUNT] = {0, 0, 255, 0};
    unsigned int noConnectionColors[COLORS_AMOUNT] = {255, 0, 0, 0};
    unsigned int updateInProgressColors[COLORS_AMOUNT] = {255, 0, 255, 0};
    unsigned int noColors[COLORS_AMOUNT] = {0, 0, 0, 0};

    // Hotword timer API
    void initHotwordResetTimer();

    void startHotwordTimer();

    // Setup and utility methods
    void initMatrixCore();

    unsigned int *adjustColors(const unsigned int *colors) const;

    void changeEverloopColors(const unsigned int *colors, unsigned int renderingLimit);

public:
    // These constants are used for tasks locking in MatrixVoiceFacade
    static const unsigned long AUDIO_STREAM_BLOCK_TIME;
    static const unsigned long EVERLOOP_BLOCK_TIME;

    // Key callbacks passed via MatrixVoiceFacade for managing LEDs and Mics
    TaskCallbackFunc everloopCallbackFn;
    TaskCallbackFunc audioStreamCallbackFn;

    MatrixVoiceHandler();

    // Hotword API
    void resetHotword();

    bool isHotwordDetected() const;

    void changeHotwordState(bool isHotwordDetected);

    // Everloop renderers
    void renderEverloop(bool isConnected);

    void renderOTAUpdateProgress(unsigned int progress);

    // Microphones handlers
    void readAudioData(const AudioCallbackFunc &audioCallback);

    bool shouldSendAudio() const;

    void changeAudioState(bool shouldSendAudio);

    void changeAudioGain(uint16_t gain);

    void changeAudioRate(uint32_t rate);

    // LEDs' task handlers
    void initEverloopTask(TaskCallbackFunc everloopCallbackFn);

    void acquireEverloop();

    void releaseEverloop();

    void waitForEverloop();

    // Microphones' task handlers
    void initAudioStreamingTask(TaskCallbackFunc audioStreamCallbackFn);

    void acquireAudioStream();

    void releaseAudioStream();

    void waitForAudioStream();

    /**
     * These 2 static wrappers are required for RTOS tasks manipulation in OO manner.
     * Otherwise, it's impossible to pass a member function callback into xTaskCreate API
     */
    static void everloopTaskWrapper(void *_params) {
        auto *instance = static_cast<MatrixVoiceHandler *>(_params);
        instance->everloopCallbackFn();
    }

    static void audioStreamingTaskWrapper(void *_params) {
        auto *instance = static_cast<MatrixVoiceHandler *>(_params);
        instance->audioStreamCallbackFn();
    }

    // Static callback for RTOS timer API wrapping in OO manner.
    static void hotwordCallbackWrapper(TimerHandle_t _handle) {
        auto *instance = static_cast<MatrixVoiceHandler *>(pvTimerGetTimerID(_handle));
        instance->resetHotword();
    }
};

#endif
