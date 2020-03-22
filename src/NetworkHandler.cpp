#include "NetworkHandler.hpp"

const long NetworkHandler::WIFI_CONNECTION_DELAY = 5000;
const long NetworkHandler::WIFI_RECONNECT_ATTEMPTS = 10;

const std::string NetworkHandler::AUDIO_FRAME_TOPIC = PID + std::string("/audioServer/audioFrame");
const std::string NetworkHandler::TOGLE_OFF_TOPIC = PID + std::string("/hotword/toggleOff");
const std::string NetworkHandler::TOGGLE_ON_TOPIC = PID + std::string("/hotword/toggleOn");
const std::string NetworkHandler::DEBUG_TOPIC = PID + std::string("/debug");
const std::string NetworkHandler::RESTART_TOPIC = PID + std::string("/restart");
const std::string NetworkHandler::TRANSCRIBE_TOPIC = PID + std::string("/finalTranscribe");

const String NetworkHandler::ASYNC_CLIENT_ID = "MatrixVoiceAsync";
const String NetworkHandler::SYNC_CLIENT_ID = "MatrixVoiceSync";
const unsigned int NetworkHandler::JSON_BUFFER_SIZE = 300;
const unsigned long NetworkHandler::NETWORK_RECONNECT_TIMEOUT = 2000;

const std::string NetworkHandler::OTA_PASSWORD_HASH_KEY = "passwordHash";

NetworkHandler::NetworkHandler(MatrixVoiceHandler *_matrixVoiceHandler) {
  isWiFiConnected = false;
  matrixVoiceHandler = _matrixVoiceHandler;
}

NetworkHandler *NetworkHandler::init() {
  initMqttClients();
  initWiFi();
  initOTA();
  return this;
}

void NetworkHandler::mqttConnectHandler(bool isSessionPresent) {
  Serial.println("[ESP] Connected to async MQTT");
  asyncMqttClient.subscribe(TOGLE_OFF_TOPIC.c_str(), 0);
  asyncMqttClient.subscribe(TOGGLE_ON_TOPIC.c_str(), 0);
  asyncMqttClient.subscribe(RESTART_TOPIC.c_str(), 0);
  asyncMqttClient.subscribe(TRANSCRIBE_TOPIC.c_str(), 0);
}

void NetworkHandler::mqttDisconnectHandler(AsyncMqttClientDisconnectReason reason) {
  Serial.println("[ESP] Disconnected from async MQTT");
  if (isWiFiConnected) {
    startMqttReconnectTimer();
  }
}

void NetworkHandler::connectToAsyncMqtt() {
  if (isWiFiConnected && !asyncMqttClient.connected()) {
    Serial.println("[ESP] Connecting to async MQTT...");
    const String clientId = ASYNC_CLIENT_ID + "-" + String(random(0xffff), HEX);
    asyncMqttClient.setClientId(clientId.c_str());
    asyncMqttClient.connect();
  }
}

void NetworkHandler::connectToSyncMqtt() {
  if (isWiFiConnected && !syncMqttClient.connected()) {
    Serial.println("[ESP] Connecting to sync MQTT...");
    const String clientId = SYNC_CLIENT_ID + "-" + String(random(0xffff), HEX);
    if (syncMqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("[ESP] Connected to sync MQTT");
      matrixVoiceHandler->changeAudioState(true);
    } else {
      Serial.println("[ESP] Cannot connect to sync MQTT");
    }
  } else {
    Serial.println("[ESP] WI-FI is not connected or sync MQTT is already connected");
  }
}

bool NetworkHandler::hasValue(const std::string inputString, const std::string searchString) {
  return inputString.find(searchString) != std::string::npos;
}

void NetworkHandler::mqttMessageHandler(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t length, size_t index, size_t total) {
  if (length + index == total) {
    const std::string convertedTopic(topic);
    const std::string convertedPayload(payload);

    if ((hasValue(convertedTopic, "finalTranscribe") && matrixVoiceHandler->isHotwordDetected()) || hasValue(convertedTopic, "toggleOn")) {
      matrixVoiceHandler->changeHotwordState(false);
      Serial.println("[ESP] Received final transcribe");
      aquireEverloop();
    } else if (hasValue(convertedTopic, "toggleOff")) {
      matrixVoiceHandler->changeHotwordState(true);
      aquireEverloop();
    } else if (hasValue(convertedTopic, RESTART_TOPIC.c_str())) {
      StaticJsonDocument<JSON_BUFFER_SIZE> jsonBuffer;
      DeserializationError error = deserializeJson(jsonBuffer, convertedPayload.c_str());
      if (!error) {
        JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
        if (jsonRoot.containsKey(OTA_PASSWORD_HASH_KEY) && jsonRoot[OTA_PASSWORD_HASH_KEY] == OTA_PASS_HASH) {
          ESP.restart();
        }
      } else {
        Serial.println("[ESP] Unable to restart ESP");
      }
    }
  }
}

void NetworkHandler::initMqttClients() {
  initMqttReconnectTimer();

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

  syncMqttClient.setClient(wifiClient);
  syncMqttClient.setServer(MQTT_IP, MQTT_PORT);
}

// Workaround for passing static callback into xTimerCreate API
static void mqttCallbackWrapper(TimerHandle_t _handle) {
  NetworkHandler *instance = static_cast<NetworkHandler *>(pvTimerGetTimerID(_handle));
  instance->connectToAsyncMqtt();
}

void NetworkHandler::initMqttReconnectTimer() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(NETWORK_RECONNECT_TIMEOUT), pdFALSE, this,
                                    mqttCallbackWrapper);
  Serial.println("[ESP] Created mqtt reconnect timer");
}

void NetworkHandler::startMqttReconnectTimer() {
  xTimerStart(mqttReconnectTimer, 0);
}

void NetworkHandler::stopMqttReconnectTimer() {
  xTimerStop(mqttReconnectTimer, 0);
}

void NetworkHandler::keepSyncMqttClientAlive() {
  syncMqttClient.loop();
}

bool NetworkHandler::isSyncMqttClientConnected() {
  return syncMqttClient.connected();
}

void NetworkHandler::publishSync(const char *topic, const uint8_t *payload, unsigned int length) {
  syncMqttClient.publish(topic, payload, length);
}

// WiFI API
// Workaround for passing static callback into xTimerCreate API
static void wifiCallbackWrapper(TimerHandle_t _handle) {
  NetworkHandler *instance = static_cast<NetworkHandler *>(pvTimerGetTimerID(_handle));
  instance->connectToWiFi();
}

void NetworkHandler::initWiFiReconnectTimer() {
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(NETWORK_RECONNECT_TIMEOUT), pdFALSE, this,
                                    wifiCallbackWrapper);
  Serial.println("[ESP] Created Wi-Fi reconnect timer");
}

void NetworkHandler::startWiFIReconnectTimer() {
  xTimerStart(wifiReconnectTimer, 0);
}

void NetworkHandler::stopWiFiReconnectTimer() {
  xTimerStop(wifiReconnectTimer, 0);
}

void NetworkHandler::wifiEventHandler(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("[ESP] Received IP address: ");
      Serial.println(WiFi.localIP());
      isWiFiConnected = true;
      connectToSyncMqtt();
      connectToAsyncMqtt();
      aquireEverloop();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("[ESP] Disconnected from Wi-Fi router");
      stopMqttReconnectTimer();
      startWiFIReconnectTimer();
      isWiFiConnected = false;
      matrixVoiceHandler->changeAudioState(false);
      matrixVoiceHandler->changeHotwordState(false);
      aquireEverloop();
      break;
    default:
      break;
  }
}

void NetworkHandler::connectToWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

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

bool NetworkHandler::isWiFiClientConnected() {
  return isWiFiConnected;
}

// OTA API
void NetworkHandler::initOTA() {
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setPasswordHash(OTA_PASS_HASH);
  ArduinoOTA
      .onStart([this]() {
        this->isUpdateInProgess = true;
        this->isWiFiConnected = false;
        this->matrixVoiceHandler->changeAudioState(false);
        this->matrixVoiceHandler->changeHotwordState(false);
        releaseAudioStream();
        releaseEverloop();
      })
      .onProgress([this](unsigned int progress, unsigned int total) {
        unsigned int estimatedProgress = progress / (total / 100);
        this->matrixVoiceHandler->renderOTAUpdateProgress(estimatedProgress);
      })
      .onEnd([this]() {
        this->isUpdateInProgess = false;
      });
  ArduinoOTA.begin();
}

void NetworkHandler::trackOTAUpdates() {
  ArduinoOTA.handle();
}

bool NetworkHandler::isOTAUpdateInProgress() {
  return isUpdateInProgess;
}
