#ifndef NETWORK_HANDLER_HPP
#define NETWORK_HANDLER_HPP

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <AsyncMqttClient.h>

#include "MatrixVoiceHandler.hpp"

class NetworkHandler {
  private:
    // Wi-Fi
    static const unsigned long NETWORK_RECONNECT_TIMEOUT;
    static const long WIFI_CONNECTION_DELAY;
    static const long WIFI_RECONNECT_ATTEMPTS;

    WiFiClient wifiClient;
    TimerHandle_t wifiReconnectTimer;
    bool isWiFiConnected;

    void initWiFi();
    void wifiEventHandler(WiFiEvent_t event);
    void initWiFiReconnectTimer();
    void startWiFIReconnectTimer();
    void stopWiFiReconnectTimer();
    
    // MQTT
    static const unsigned int JSON_BUFFER_SIZE;
    static const String ASYNC_CLIENT_ID;
    static const String SYNC_CLIENT_ID;

    AsyncMqttClient asyncMqttClient;
    PubSubClient syncMqttClient;
    TimerHandle_t mqttReconnectTimer;

    void initMqttClients();
    void connectToSyncMqtt();
    void initMqttReconnectTimer();
    void startMqttReconnectTimer();
    void stopMqttReconnectTimer();
    void mqttConnectHandler(bool isSessionPresent);
    void mqttDisconnectHandler(AsyncMqttClientDisconnectReason reason);
    void mqttMessageHandler(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t length, size_t index, size_t total);
    bool hasValue(const std::string inputString, const std::string searchString);

    // OTA
    static const std::string OTA_PASSWORD_HASH_KEY;
    bool isUpdateInProgess;
    
    void initOTA();

    // MatrixVoice
    MatrixVoiceHandler *matrixVoiceHandler;

  public:
    static const std::string AUDIO_FRAME_TOPIC;
    static const std::string TOGLE_OFF_TOPIC;
    static const std::string TOGGLE_ON_TOPIC;
    static const std::string DEBUG_TOPIC;
    static const std::string RESTART_TOPIC;
    static const std::string TRANSCRIBE_TOPIC;

    NetworkHandler(MatrixVoiceHandler *_matrixVoiceHandler);
    NetworkHandler *init();
    // WiFi
    bool isWiFiClientConnected();
    void connectToWiFi();

    // OTA
    void trackOTAUpdates();
    bool isOTAUpdateInProgress();

    // MQTT
    void connectToAsyncMqtt();
    void keepSyncMqttClientAlive();
    bool isSyncMqttClientConnected();
    void publishSync(const char* topic, const uint8_t* payload, unsigned int length);
};

#endif
