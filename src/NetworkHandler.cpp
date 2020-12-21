#include "NetworkHandler.hpp"

// Wi-Fi settings
const long NetworkHandler::WIFI_CONNECTION_DELAY = 1000;
const long NetworkHandler::WIFI_RECONNECT_ATTEMPTS = 10;

__unused __unused  // Use this topic if you want to print some debug info (not used right now in favor of Serial log)
const std::string NetworkHandler::DEBUG_TOPIC = PID + std::string("/debug");
// A special topic for restarting ESP32
const std::string NetworkHandler::RESTART_TOPIC = PID + std::string("/restart");
// Microphones settings
const std::string NetworkHandler::AUDIO_GAIN_TOPIC = PID + std::string("/gain");
const std::string NetworkHandler::AUDIO_RATE_TOPIC = PID + std::string("/rate");
const std::string NetworkHandler::MUTE_TOPIC = PID + std::string("/mute");
// Common JSON keys expected in a payload
const std::string NetworkHandler::COMMON_PAYLOAD_KEY = "value";
const std::string NetworkHandler::TRANSCRIBE_PAYLOAD_KEY = "text";

// MQTT settings
const String NetworkHandler::ASYNC_CLIENT_ID = "MatrixVoiceAsync";
const unsigned int NetworkHandler::JSON_BUFFER_SIZE = 1024;
const unsigned long NetworkHandler::NETWORK_RECONNECT_TIMEOUT = 2000;

// A payload key used for restarting ESP32 via async MQTT
const std::string NetworkHandler::OTA_PASSWORD_HASH_KEY = "passwordHash";

NetworkHandler::NetworkHandler(MatrixVoiceHandler *_matrixVoiceHandler) {
    isWiFiConnected = false;
    isSocketClientConnected = false;
    matrixVoiceHandler = _matrixVoiceHandler;
}

/**
 * Setup Wi-Fi, MQTT and OTA.
 */
void NetworkHandler::setup() {
    initMqttClient();
    initWiFi();
    initOTA();
}

/**
 * When we establish connection via async MQTT, we need to subscribe to common topics.
 */
void NetworkHandler::mqttConnectHandler(__unused bool isSessionPresent) {
    Serial.println("[ESP] Connected to async MQTT");
    asyncMqttClient.subscribe(RESTART_TOPIC.c_str(), 0);
    asyncMqttClient.subscribe(AUDIO_RATE_TOPIC.c_str(), 0);
    asyncMqttClient.subscribe(AUDIO_GAIN_TOPIC.c_str(), 0);
    asyncMqttClient.subscribe(MUTE_TOPIC.c_str(), 0);
}

/**
 * When we are disconnected from async MQTT, we start reconnect timer.
 */
void NetworkHandler::mqttDisconnectHandler(__unused AsyncMqttClientDisconnectReason reason) {
    Serial.println("[ESP] Disconnected from async MQTT");
    if (isWiFiConnected) {
        startMqttReconnectTimer();
    }
}

/**
 * Connect to MQTT broker via async client. Use it for a common messaging stuff.
 */
void NetworkHandler::connectToAsyncMqtt() {
    if (isWiFiConnected && !asyncMqttClient.connected()) {
        const String clientId = ASYNC_CLIENT_ID + "-" + String(random(0xffff), HEX);
        asyncMqttClient.setClientId(clientId.c_str());
        asyncMqttClient.connect();
    }
}

/**
 * A helper method for topics' / payloads' comparison.
 */
bool NetworkHandler::hasValue(const std::string &inputString, const std::string &searchString) {
    return inputString.find(searchString) != std::string::npos;
}

/**
 * Main async MQTT client callback for common messages' processing.
 */
void
NetworkHandler::mqttMessageHandler(char *topic, char *payload, __unused AsyncMqttClientMessageProperties properties,
                                   size_t length, __unused size_t index, __unused size_t total) {
    const std::string convertedTopic(topic);
    StaticJsonDocument<JSON_BUFFER_SIZE> jsonBuffer;
    DeserializationError error = deserializeJson(jsonBuffer, payload, length);

    // If we want to restart ESP32 for some reason, we have to provide a hashed password in a payload
    if (hasValue(convertedTopic, RESTART_TOPIC) && error.code() == DeserializationError::Ok) {
        JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
        if (jsonRoot.containsKey(OTA_PASSWORD_HASH_KEY) && jsonRoot[OTA_PASSWORD_HASH_KEY] == OTA_PASS_HASH) {
            ESP.restart();
        }
    } else if (hasValue(convertedTopic, MUTE_TOPIC) && error.code() == DeserializationError::Ok) {
        // Emulate microphones mute
        JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
        if (jsonRoot.containsKey(COMMON_PAYLOAD_KEY)) {
            bool shouldMute = jsonRoot[COMMON_PAYLOAD_KEY];
            matrixVoiceHandler->changeAudioState(!shouldMute);
            Serial.println("[ESP] Updated audio state");
        }
    } else if (hasValue(convertedTopic, AUDIO_GAIN_TOPIC) && error.code() == DeserializationError::Ok) {
        // Change microphones gain
        JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
        if (jsonRoot.containsKey(COMMON_PAYLOAD_KEY)) {
            uint16_t gain = jsonRoot[COMMON_PAYLOAD_KEY];
            matrixVoiceHandler->changeAudioGain(gain);
            Serial.println("[ESP] Updated gain");
        }
    } else if (hasValue(convertedTopic, AUDIO_RATE_TOPIC) && error.code() == DeserializationError::Ok) {
        // Change audio sampling rate
        JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
        if (jsonRoot.containsKey(COMMON_PAYLOAD_KEY)) {
            uint32_t rate = jsonRoot[COMMON_PAYLOAD_KEY];
            matrixVoiceHandler->changeAudioRate(rate);
            Serial.println("[ESP] Updated sampling rate");
        }
    }
}

/**
 * Setup async MQTT client.
 */
void NetworkHandler::initMqttClient() {
    initMqttReconnectTimer();

    // Async client configuration
    asyncMqttClient.onConnect([this](bool isSessionPresent) {
        this->mqttConnectHandler(isSessionPresent);
    });
    asyncMqttClient.onDisconnect([this](AsyncMqttClientDisconnectReason reason) {
        this->mqttDisconnectHandler(reason);
    });
    asyncMqttClient.onMessage(
            [this](char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t length, size_t index,
                   size_t total) {
                this->mqttMessageHandler(topic, payload, properties, length, index, total);
            });
    asyncMqttClient.setServer(MQTT_IP, MQTT_PORT);
    asyncMqttClient.setCredentials(MQTT_USER, MQTT_PASS);
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

void NetworkHandler::stopMqttReconnectTimer() {
    xTimerStop(mqttReconnectTimer, 0);
}

/**
 * Setup WS client.
 */
void NetworkHandler::initSocket() {
    webSocketsClient.begin(WS_HOST, WS_PORT);
    webSocketsClient.onEvent([this](WStype_t type, uint8_t *payload, size_t length) {
        this->handleWebSocketEvent(type, payload, length);
    });
    webSocketsClient.setReconnectInterval(WS_RECONNECT_INTERVAL);
}

/**
 * WS polling, which is called within Arduino loop.
 */
void NetworkHandler::keepSocketClientAlive() {
    webSocketsClient.loop();
}

/**
 * WS publisher, which is used for sending audio chunks to Vosk server.
 */
void NetworkHandler::publishData(uint8_t *payload, size_t length) {
    webSocketsClient.sendBIN(payload, length);
}

/**
 * WS events handler.
 */
void NetworkHandler::handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("[ESP] Disconnected from websocket server");
            isSocketClientConnected = false;
            matrixVoiceHandler->changeAudioState(false);
            matrixVoiceHandler->acquireEverloop();
            break;
        case WStype_CONNECTED:
            Serial.println("[ESP] Connected to websocket server");
            isSocketClientConnected = true;
            matrixVoiceHandler->changeAudioState(true);
            matrixVoiceHandler->acquireEverloop();
            matrixVoiceHandler->acquireAudioStream();
            break;
        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
        case WStype_PING:
        case WStype_PONG:
        case WStype_BIN:
            break;
        case WStype_TEXT:
            StaticJsonDocument<JSON_BUFFER_SIZE> jsonBuffer;
            DeserializationError error = deserializeJson(jsonBuffer, payload, length);
            if (!error) {
                JsonObject jsonRoot = jsonBuffer.as<JsonObject>();
                if (jsonRoot.containsKey(TRANSCRIBE_PAYLOAD_KEY) && matrixVoiceHandler->isHotwordDetected()) {
                    matrixVoiceHandler->changeHotwordState(false);
                    matrixVoiceHandler->acquireEverloop();
                }
            }
            break;
    }
}

/**
 * External check if WS client is still connected.
 */
bool NetworkHandler::isSocketConnected() const {
    return isSocketClientConnected;
}

/**
 * Wi-FI connect / reconnect events handler.
 */
void NetworkHandler::wifiEventHandler(WiFiEvent_t event) {
    switch (event) {
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.println("[ESP] Received IP address: " + WiFi.localIP().toString());
            isWiFiConnected = true;
            isUpdateInProgress = false;
            initSocket();
            connectToAsyncMqtt();
            break;
            // If we are disconnected, we need to block audio streaming and update LEDs
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("[ESP] Disconnected from Wi-Fi router");
            isWiFiConnected = false;
            isSocketClientConnected = false;
            matrixVoiceHandler->changeAudioState(false);
            matrixVoiceHandler->changeHotwordState(false);
            matrixVoiceHandler->acquireEverloop();
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

        WiFi.onEvent([this](WiFiEvent_t event, system_event_info_t info) {
            this->wifiEventHandler(event);
        });

        WiFiClass::mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        WiFi.setSleep(false);

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
bool NetworkHandler::isWiFiClientConnected() const {
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
                this->isUpdateInProgress = true;
                if (this->isSocketClientConnected) {
                    this->webSocketsClient.disconnect();
                }
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
bool NetworkHandler::isOTAUpdateInProgress() const {
    return isUpdateInProgress;
}
