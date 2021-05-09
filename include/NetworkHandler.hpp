#ifndef NETWORK_HANDLER_HPP
#define NETWORK_HANDLER_HPP

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <WebSocketsClient.h>
#include <sys/cdefs.h>
#include <rom/queue.h>

#include "MatrixVoiceHandler.hpp"

class NetworkHandler {
private:
    // Wi-Fi settings
    static const unsigned long NETWORK_RECONNECT_TIMEOUT;
    static const long WIFI_CONNECTION_DELAY;
    static const long WIFI_RECONNECT_ATTEMPTS;

    WiFiClient wifiClient;
    bool isWiFiConnected;

    void initWiFi();

    void wifiEventHandler(WiFiEvent_t event);

    // MQTT settings
    static const unsigned int JSON_BUFFER_SIZE;
    static const String ASYNC_CLIENT_ID;

    AsyncMqttClient asyncMqttClient;
    TimerHandle_t mqttReconnectTimer{};

    void initMqttClient();

    void initMqttReconnectTimer();

    void startMqttReconnectTimer();

    void stopMqttReconnectTimer();

    void mqttConnectHandler(__unused bool isSessionPresent);

    void mqttDisconnectHandler(__unused AsyncMqttClientDisconnectReason reason);

    void
    mqttMessageHandler(char *topic, char *payload, __unused AsyncMqttClientMessageProperties properties, size_t length,
                       __unused  size_t index, __unused size_t total);

    static bool hasValue(const std::string &inputString, const std::string &searchString);

    // WebSocket settings
    WebSocketsClient webSocketsClient;
    bool isSocketClientConnected;

    void initSocket();

    void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length);

    // OTA settings
    static const std::string OTA_PASSWORD_HASH_KEY;
    bool isUpdateInProgress{};

    void initOTA();

    // MatrixVoice
    MatrixVoiceHandler *matrixVoiceHandler;

public:
    // Common MQTT topics
    __unused  static const std::string DEBUG_TOPIC;
    static const std::string RESTART_TOPIC;
    static const std::string AUDIO_GAIN_TOPIC;
    static const std::string AUDIO_RATE_TOPIC;
    static const std::string MUTE_TOPIC;
    static const std::string COMMON_PAYLOAD_KEY;
    static const std::string TRANSCRIBE_PAYLOAD_KEY;

    explicit NetworkHandler(MatrixVoiceHandler *_matrixVoiceHandler);

    void setup();

    // WiFi API
    bool isWiFiClientConnected() const;

    // OTA API
    static void trackOTAUpdates();

    bool isOTAUpdateInProgress() const;

    // MQTT API
    void connectToAsyncMqtt();

    void publishData(uint8_t *payload, size_t length);

    // Static callback wrapper is required for using xTimerCreate API in OO manner.
    static void mqttCallbackWrapper(TimerHandle_t _handle) {
        auto *instance = static_cast<NetworkHandler *>(pvTimerGetTimerID(_handle));
        instance->connectToAsyncMqtt();
    }

    // WebSocket API
    void keepSocketClientAlive();

    bool isSocketConnected() const;
};

#endif
