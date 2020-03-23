#include "NetworkHandler.hpp"

// Wi-Fi settings
const long NetworkHandler::WIFI_CONNECTION_DELAY = 5000;
const long NetworkHandler::WIFI_RECONNECT_ATTEMPTS = 10;

// Topic required for streaming audio chunks to Kaldi server
const std::string NetworkHandler::VOICE_STREAM_TOPIC = PID + std::string("/stream/voice");
// This topic may change hotword flag to trigger either wakeword detection or streaming to Kaldi
const std::string NetworkHandler::HOTWORD_TOPIC = PID + std::string("/hotword");
// Use this topic to block or unblock audio streaming
const std::string NetworkHandler::MUTE_TOPIC = PID + std::string("/mute");
// Use this topic if you want to print some debug info (not used right now in favor of Serial log)
const std::string NetworkHandler::DEBUG_TOPIC = PID + std::string("/debug");
// A special topic for restarting ESP32
const std::string NetworkHandler::RESTART_TOPIC = PID + std::string("/restart");
// This topic is used for receiving voice transcription from Kaldi server
const std::string NetworkHandler::TRANSCRIBE_TOPIC = PID + std::string("/finalTranscribe");

// MQTT settings
const String NetworkHandler::ASYNC_CLIENT_ID = "MatrixVoiceAsync";
const String NetworkHandler::SYNC_CLIENT_ID = "MatrixVoiceSync";
const std::string NetworkHandler::COMMON_PAYLOAD_KEY = "value";
const unsigned int NetworkHandler::JSON_BUFFER_SIZE = 300;
const unsigned long NetworkHandler::NETWORK_RECONNECT_TIMEOUT = 2000;

// A payload key used for restarting ESP32 via async MQTT
const std::string NetworkHandler::OTA_PASSWORD_HASH_KEY = "passwordHash";

NetworkHandler::NetworkHandler(MatrixVoiceHandler *_matrixVoiceHandler) {
  isWiFiConnected = false;
  matrixVoiceHandler = _matrixVoiceHandler;
}

/**
 * Start Wi-Fi, MQTT and OTA.
 */
void NetworkHandler::setup() {
  initMqttClients();
  initWiFi();
  initOTA();
}

/**
 * When we establish connection via async MQTT, we need to subscribe to common topics.
 */
void NetworkHandler::mqttConnectHandler(bool isSessionPresent) {
  Serial.println("[ESP] Connected to async MQTT");
  asyncMqttClient.subscribe(HOTWORD_TOPIC.c_str(), 0);
  asyncMqttClient.subscribe(MUTE_TOPIC.c_str(), 0);
  asyncMqttClient.subscribe(RESTART_TOPIC.c_str(), 0);
  asyncMqttClient.subscribe(TRANSCRIBE_TOPIC.c_str(), 0);
}

/**
 * When we are disconnected from async MQTT, we start reconnect timer.
 */
void NetworkHandler::mqttDisconnectHandler(AsyncMqttClientDisconnectReason reason) {
  Serial.println("[ESP] Disconnected from async MQTT");
  if (isWiFiConnected) {
    startMqttReconnectTimer();
  }
}

/**
 * Connect to MQTT broker via async client. This client is used only for a common messaging stuff.
 * See sync client for getting info about audio streaming.
 */
void NetworkHandler::connectToAsyncMqtt() {
  if (isWiFiConnected && !asyncMqttClient.connected()) {
    Serial.println("[ESP] Connecting to async MQTT...");
    const String clientId = ASYNC_CLIENT_ID + "-" + String(random(0xffff), HEX);
    asyncMqttClient.setClientId(clientId.c_str());
    asyncMqttClient.connect();
  }
}

/**
 * Connect to MQTT broker via sync client. This client is used for sending audio stream to Kaldi server.
 * We can't reuse the a single client both for streaming and common messaging.
 */
void NetworkHandler::connectToSyncMqtt() {
  if (isWiFiConnected && !syncMqttClient.connected()) {
    Serial.println("[ESP] Connecting to sync MQTT...");
    const String clientId = SYNC_CLIENT_ID + "-" + String(random(0xffff), HEX);
    if (syncMqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("[ESP] Connected to sync MQTT");
      // Let Matrix Voice handler know if we can start streaming
      matrixVoiceHandler->changeAudioState(true);
    } else {
      Serial.println("[ESP] Cannot connect to sync MQTT");
    }
  } else {
    Serial.println("[ESP] WI-FI is not connected or sync MQTT is already connected");
  }
}

/**
 * A helper method for topics' / payloads' comparison.
 */
bool NetworkHandler::hasValue(const std::string inputString, const std::string searchString) {
  return inputString.find(searchString) != std::string::npos;
}

/**
 * Main async MQTT client callback for common messages' processing.
 */
void NetworkHandler::mqttMessageHandler(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t length, size_t index, size_t total) {
  if (length + index == total) {
    const std::string convertedTopic(topic);
    StaticJsonDocument<JSON_BUFFER_SIZE> jsonBuffer;
    DeserializationError error = deserializeJson(jsonBuffer, payload, length);

    // If we receive a trascription from Kaldi server, we have to go back to wakeword detection state
    if (hasValue(convertedTopic, "finalTranscribe")) {
      Serial.println("[ESP] Received final transcribe");
      matrixVoiceHandler->changeHotwordState(false);
      matrixVoiceHandler->aquireEverloop();
      // In case of any unexpected behaviour with streaming, we can still change hotword flag manually
    } else if (hasValue(convertedTopic, "hotword") && error.code() == DeserializationError::Ok) {
      JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
      if (jsonRoot.containsKey(COMMON_PAYLOAD_KEY)) {
        bool isDetected = jsonRoot[COMMON_PAYLOAD_KEY];
        matrixVoiceHandler->changeHotwordState(isDetected);
        matrixVoiceHandler->aquireEverloop();
        Serial.println("[ESP] Updated hotword state");
      }
      // If we want to mute / unmute mics, we just change a flag to avoid reading and streaming data
    } else if (hasValue(convertedTopic, "mute") && error.code() == DeserializationError::Ok) {
      JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
      if (jsonRoot.containsKey(COMMON_PAYLOAD_KEY)) {
        bool shouldMute = jsonRoot[COMMON_PAYLOAD_KEY];
        matrixVoiceHandler->changeAudioState(!shouldMute);
        Serial.println("[ESP] Updated audio state");
      }
      // If we want to restart ESP32 for some reason, we have to provide a hashed password in a payload
    } else if (hasValue(convertedTopic, RESTART_TOPIC.c_str()) && error.code() == DeserializationError::Ok) {
      JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
      if (jsonRoot.containsKey(OTA_PASSWORD_HASH_KEY) && jsonRoot[OTA_PASSWORD_HASH_KEY] == OTA_PASS_HASH) {
        ESP.restart();
      }
    }
  }
}

/**
 * Setup sync and async MQTT clients.
 */
void NetworkHandler::initMqttClients() {
  initMqttReconnectTimer();

  // Async clients configuration
  asyncMqttClient.onConnect([this](bool isSessionPresent) {
    this->mqttConnectHandler(isSessionPresent);
  });
  asyncMqttClient.onDisconnect([this](AsyncMqttClientDisconnectReason reason) {
    this->mqttDisconnectHandler(reason);
  });
  asyncMqttClient.onMessage([this](char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t length, size_t index, size_t total) {
    this->mqttMessageHandler(topic, payload, properties, length, index, total);
  });
  asyncMqttClient.setServer(MQTT_IP, MQTT_PORT);
  asyncMqttClient.setCredentials(MQTT_USER, MQTT_PASS);

  // Sync client configuration
  syncMqttClient.setClient(wifiClient);
  syncMqttClient.setServer(MQTT_IP, MQTT_PORT);
}

/**
 * In case if we lose async MQTT connection, this timer will help to reconnect automatically.
 */
void NetworkHandler::initMqttReconnectTimer() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(NETWORK_RECONNECT_TIMEOUT), pdFALSE, this,
                                    mqttCallbackWrapper);
  Serial.println("[ESP] Created mqtt reconnect timer");
}

/**
 * MQTT reconnect timer is started within disconnect callback.
 */
void NetworkHandler::startMqttReconnectTimer() {
  xTimerStart(mqttReconnectTimer, 0);
}

/**
 * Common sync MQTT polling, which is called within Arduino loop.
 */
void NetworkHandler::keepSyncMqttClientAlive() {
  syncMqttClient.loop();
}

/**
 * External check if sync MQTT is still connected.
 */
bool NetworkHandler::isSyncMqttClientConnected() {
  return syncMqttClient.connected();
}

/**
 * Main sync MQTT publisher, which is used for sending audio chunks to Kaldi server.
 */
void NetworkHandler::publishSync(const char *topic, const uint8_t *payload, unsigned int length) {
  syncMqttClient.publish(topic, payload, length);
}

/**
 * Wi-FI connect / reconnect events handler.
 */
void NetworkHandler::wifiEventHandler(WiFiEvent_t event) {
  switch (event) {
    // When we connect to Wi-Fi, we can proceed with MQTT and change LEDs state
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("[ESP] Received IP address: ");
      Serial.println(WiFi.localIP());
      isWiFiConnected = true;
      isUpdateInProgess = false;
      connectToSyncMqtt();
      connectToAsyncMqtt();
      matrixVoiceHandler->aquireEverloop();
      break;
    // If we are disconnected, we need to block audio streaming and update LEDs
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("[ESP] Disconnected from Wi-Fi router");
      isWiFiConnected = false;
      matrixVoiceHandler->changeAudioState(false);
      matrixVoiceHandler->changeHotwordState(false);
      matrixVoiceHandler->aquireEverloop();
      break;
    default:
      break;
  }
}

/**
 * Setup Wi-FI connection.
 */
void NetworkHandler::initWiFi() {
  if (!isWiFiConnected) {
    Serial.println("[ESP] WI-FI is not connected, setting up...");
    unsigned int attempt = 0;

    WiFi.mode(WIFI_STA);
    WiFi.onEvent([this](WiFiEvent_t event, system_event_info_t info) {
      this->wifiEventHandler(event);
    });
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      delay(WIFI_CONNECTION_DELAY);
      if (attempt > WIFI_RECONNECT_ATTEMPTS) {
        Serial.println("[ESP] Cannot connect to Wi-Fi router, restarting...");
        ESP.restart();
      }

      attempt++;
    }
  }
}

/**
 * External check if Wi-Fi client is still connected.
 */
bool NetworkHandler::isWiFiClientConnected() {
  return isWiFiConnected;
}

/**
 * Common OTA configuration to be able to update code without wires.
 */
void NetworkHandler::initOTA() {
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setPasswordHash(OTA_PASS_HASH);
  ArduinoOTA
      // Reset all the states while updating
      .onStart([this]() {
        this->isUpdateInProgess = true;
        this->isWiFiConnected = false;
        this->matrixVoiceHandler->changeAudioState(false);
        this->matrixVoiceHandler->changeHotwordState(false);
        this->matrixVoiceHandler->releaseAudioStream();
        this->matrixVoiceHandler->releaseEverloop();
      })
      // Send progress to everloop API
      .onProgress([this](unsigned int progress, unsigned int total) {
        unsigned int estimatedProgress = progress / (total / 100);
        this->matrixVoiceHandler->renderOTAUpdateProgress(estimatedProgress);
      });
  ArduinoOTA.begin();
}

/**
 * OTA updates checking in a main Arduino loop.
 */
void NetworkHandler::trackOTAUpdates() {
  ArduinoOTA.handle();
}

/**
 * Make sure we are not polling while OTA updates.
 */
bool NetworkHandler::isOTAUpdateInProgress() {
  return isUpdateInProgess;
}
