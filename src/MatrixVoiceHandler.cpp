#include "MatrixVoiceHandler.hpp"

// WakeNet
const esp_wn_iface_t *MatrixVoiceHandler::WAKE_NET = &WAKENET_MODEL;
const model_coeff_getter_t *MatrixVoiceHandler::MODEL_COEFF_GETTER = &WAKENET_COEFF;

// Mics
const unsigned long MatrixVoiceHandler::AUDIO_STREAM_BLOCK_TIME = 5000;
const unsigned long MatrixVoiceHandler::HOTWORD_RESET_TIMEOUT = 8000;
const int MatrixVoiceHandler::VOICE_BUFFER_SIZE = 1024;

// Everloop
const unsigned long MatrixVoiceHandler::EVERLOOP_BLOCK_TIME = 10000;
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
  model_data = WAKE_NET->create(MODEL_COEFF_GETTER, DET_MODE_90);
  _shouldSendAudio = false;
  _isHotwordDetected = false;
  brightness = 15;
}

MatrixVoiceHandler *MatrixVoiceHandler::init() {
  initHotwordResetTimer();
  initMatrixCore();
  return this;
}

// Hotword API
void MatrixVoiceHandler::resetHotword() {
  if (isHotwordDetected()) {
    changeHotwordState(false);
    aquireEverloop();
    Serial.println("[ESP] Reset hotword");
  }
}

// Workaround for passing static callback into xTimerCreate API
static void hotwordCallbackWrapper(TimerHandle_t _handle) {
  MatrixVoiceHandler *instance = static_cast<MatrixVoiceHandler *>(pvTimerGetTimerID(_handle));
  instance->resetHotword();
}

void MatrixVoiceHandler::initHotwordResetTimer() {
  hotwordTimer = xTimerCreate("hotwordTimer", pdMS_TO_TICKS(HOTWORD_RESET_TIMEOUT), pdFALSE, this,
                              hotwordCallbackWrapper);
  Serial.println("[ESP] Created hotword timer");
}

void MatrixVoiceHandler::startHotwordTimer() {
  xTimerStart(hotwordTimer, 0);
}

bool MatrixVoiceHandler::isHotwordDetected() {
  return _isHotwordDetected;
}

void MatrixVoiceHandler::changeHotwordState(bool isHotwordDetected) {
  _isHotwordDetected = isHotwordDetected;
}

// Matrix API
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

// Everloop
unsigned int *MatrixVoiceHandler::adjustColors(unsigned int *colors) {
  unsigned int *rgbwColors = new unsigned int[COLORS_AMOUNT];
  for (unsigned int i = 0; i < COLORS_AMOUNT; i++) {
    const unsigned int color = floor(brightness * colors[i] / 100);
    rgbwColors[i] = pgm_read_byte(&GAMMA8[color]);
  }
  return rgbwColors;
}

void MatrixVoiceHandler::renderOTAUpdateProgress(unsigned int estimatedUpdateProgress) {
  const unsigned int *rgbwColors = adjustColors(updateInProgressColors);
  const unsigned int *rgbwNoColors = adjustColors(noColors);
  const unsigned int ledsSize = image1d.leds.size();
  const float progress = round((estimatedUpdateProgress / 100.0f) * (float) ledsSize);

  for (int i = 0; i < ledsSize; i++) {
    matrix_hal::LedValue &currentLed = image1d.leds[i];
    if (i <= progress) {
      currentLed.Set(rgbwColors[0], rgbwColors[1], rgbwColors[2], rgbwColors[3]);
    } else {
      currentLed.Set(rgbwNoColors[0], rgbwNoColors[1], rgbwNoColors[2], rgbwNoColors[3]);
    }
    everloop.Write(&image1d);
  }

  delete rgbwColors;
  delete rgbwNoColors;
}

void MatrixVoiceHandler::renderEverloop(bool isWiFiConnected) {
  unsigned int *rgbwColors;
  const unsigned int *rgbwNoColors = adjustColors(noColors);
  const unsigned int ledsSize = image1d.leds.size();

  if (isHotwordDetected()) {
    rgbwColors = adjustColors(hotwordColors);
  } else if (!isWiFiConnected) {
    rgbwColors = adjustColors(noWiFiColors);
  } else {
    rgbwColors = adjustColors(idleColors);
  }

  // Animate LED ring
  unsigned int ledNumber = 0;
  do {
    for (int i = 0; i < ledsSize; i++) {
      matrix_hal::LedValue &currentLed = image1d.leds[i];
      if (i <= ledNumber) {
        currentLed.Set(rgbwColors[0], rgbwColors[1], rgbwColors[2], rgbwColors[3]);
      } else {
        currentLed.Set(rgbwNoColors[0], rgbwNoColors[1], rgbwNoColors[2], rgbwNoColors[3]);
      }
      everloop.Write(&image1d);
    }
    delay(40);
  } while (ledNumber++ < ledsSize);

  delete rgbwColors;
  delete rgbwNoColors;
}

// Mics
void MatrixVoiceHandler::readAudioData(void (*voiceCallbackFn)(uint8_t *voicebuffer, unsigned int bufferSize)) {
  microphoneArray.Read();

  int16_t voicebuffer[VOICE_BUFFER_SIZE];
  for (uint16_t i = 0; i < VOICE_BUFFER_SIZE; i++) {
    voicebuffer[i] = microphoneArray.Beam(i);
  }

  if (!isHotwordDetected()) {
    int resultCode = WAKE_NET->detect(model_data, voicebuffer);
    if (resultCode > 0) {
      Serial.println("[ESP] Detected hotword");
      changeHotwordState(true);
      aquireEverloop();
      startHotwordTimer();
    }
  }

  if (isHotwordDetected()) {
    voiceCallbackFn((uint8_t *)voicebuffer, VOICE_BUFFER_SIZE);
  }
}

void MatrixVoiceHandler::changeAudioState(bool shouldSendAudio) {
  _shouldSendAudio = shouldSendAudio;
}

bool MatrixVoiceHandler::shouldSendAudio() {
  return _shouldSendAudio;
}
