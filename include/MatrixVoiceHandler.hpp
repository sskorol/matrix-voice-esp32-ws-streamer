#ifndef MATRIX_VOICE_HANDLER_HPP
#define MATRIX_VOICE_HANDLER_HPP

#include <Arduino.h>

#include "everloop.h"
#include "everloop_image.h"
#include "microphone_array.h"
#include "microphone_core.h"
#include "wishbone_bus.h"

extern "C" {
#include "esp_wn_iface.h"
}

#include "TasksHandler.hpp"

// Mics
#define RATE 16000
#define GAIN 4

// WakeNet
#define WAKENET_COEFF get_coeff_wakeNet3_model_float
#define WAKENET_MODEL esp_sr_wakenet3_quantized

extern const esp_wn_iface_t esp_sr_wakenet3_quantized;
extern const model_coeff_getter_t get_coeff_wakeNet3_model_float;

class MatrixVoiceHandler {
 private:
  // Matrix Voice
  matrix_hal::WishboneBus bus;
  matrix_hal::MicrophoneArray microphoneArray;
  matrix_hal::Everloop everloop;
  matrix_hal::EverloopImage image1d;

  // WakeNet
  static const esp_wn_iface_t *WAKE_NET;
  static const model_coeff_getter_t *MODEL_COEFF_GETTER;
  model_iface_data_t *model_data;

  // If there's an issue receiving a voice transcription, this timer will reset hotword state
  TimerHandle_t hotwordTimer;
  static const unsigned long HOTWORD_RESET_TIMEOUT;

  // Mics
  static const int VOICE_BUFFER_SIZE;
  bool _isHotwordDetected;
  bool _shouldSendAudio;

  // Everloop
  static const uint8_t PROGMEM GAMMA8[];
  static const unsigned int COLORS_AMOUNT = 4;
  unsigned int hotwordColors[COLORS_AMOUNT] = {0, 255, 0, 0};
  unsigned int idleColors[COLORS_AMOUNT] = {0, 0, 255, 0};
  unsigned int noWiFiColors[COLORS_AMOUNT] = {255, 0, 0, 0};
  unsigned int updateInProgressColors[COLORS_AMOUNT] = {121, 32, 210, 0};
  unsigned int noColors[COLORS_AMOUNT] = {0, 0, 0, 0};
  unsigned int brightness;

  void initHotwordResetTimer();
  void startHotwordTimer();

  void initMatrixCore();
  unsigned int *adjustColors(unsigned int *colors);

 public:
  static const unsigned long AUDIO_STREAM_BLOCK_TIME;
  static const unsigned long EVERLOOP_BLOCK_TIME;

  MatrixVoiceHandler();
  MatrixVoiceHandler *init();

  // Hotword
  void resetHotword();
  bool isHotwordDetected();
  void changeHotwordState(bool isHotwordDetected);
  
  // Everloop
  void renderEverloop(bool isWiFiConnected);
  void renderOTAUpdateProgress(unsigned int progress);
  
  // Mics
  void readAudioData(void (*voiceCallbackFn)(uint8_t *voiceBuffer, unsigned int bufferSize));
  bool shouldSendAudio();
  void changeAudioState(bool shouldSendAudio);
};

#endif
