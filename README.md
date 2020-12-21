## Matrix Voice WebSockets Streamer

This repository allows you to build an offline ASR module based on WebSockets and MQTT protocols.

Note that this code was inspired by [Matrix-Voice-ESP32-MQTT-Audio-Streamer](https://github.com/Romkabouter/Matrix-Voice-ESP32-MQTT-Audio-Streamer).

Here are key differences in comparison with the original version:

- Project structure: the code was decomposed into logical blocks to increase readability and maintainability.
- MQTT: a sync client, which was initially used for audio streaming, was completely removed due instability (bugs that put ESP32 into zombie mode).
- Sockets: instead of a sync MQTT there was added a much more stable WebSockets client for voice streaming.
- Audio playback feature was completely removed due to instability and bad output quality.
- Settings: this version doesn't have such amount of customizations and doesn't persist configs in a SPIFFS.
- Everloop: there was added a new OTA animation and optimized the existing circular reaction on wake word detection event.

### Setup

- Setup RPi and laptop/PC for the first flashing as described [here](https://matrix-io.github.io/matrix-documentation/matrix-voice/esp32/).
- Import a source code into VSCode or CLion as a PlatformIO project.
- Adjust `platformio.ini` according to comments.
- Connect Matrix Voice to RPi.
- Run `./deploy [RPi_IP]`.

### Voice Streaming

This code is configured to work with [Vosk ASR](https://github.com/sskorol/asr-server) out of the box.
However, if you want to stream voice data to your own ASR engine, make sure your WS server can accept 1024-bytes chunks in a 16k sample rate.
