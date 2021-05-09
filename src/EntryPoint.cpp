#include "MatrixVoiceFacade.hpp"

#define BAUD_RATE 115200

MatrixVoiceFacade *matrixVoiceFacade;

void setup() {
    Serial.begin(BAUD_RATE);
    matrixVoiceFacade = new MatrixVoiceFacade();
}

void loop() {
    matrixVoiceFacade->keepAlive();
    delay(1);
}
