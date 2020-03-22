#ifndef MATRIX_VOICE_FACADE_HPP
#define MATRIX_VOICE_FACADE_HPP

#include "MatrixVoiceHandler.hpp"
#include "NetworkHandler.hpp"
#include "TasksHandler.hpp"

/**
 * Aggregates key handlers to provide high level API to be used in EntryPoint.
 */
class MatrixVoiceFacade {
 private:
  MatrixVoiceHandler *matrixVoiceHandler;
  NetworkHandler *networkHandler;
  
  // These callbacks mix matrix voice and network handlers' API. So they have to be located behind facade.
  void everloopTaskCallback();
  void audioStreamingTaskCallback();

 public:
  MatrixVoiceFacade();

  // Global polling API for the Arduino loop method.
  void checkForUpdates();
  void keepMqttAlive();
};

#endif
