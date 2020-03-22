#include "MatrixVoiceFacade.hpp"

MatrixVoiceFacade *matrixVoiceFacade;

void setup() {
  Serial.begin(115200);
  matrixVoiceFacade = new MatrixVoiceFacade();
}

void loop() {
  matrixVoiceFacade->checkForUpdates();
  matrixVoiceFacade->keepMqttAlive();
  delay(1);
}
