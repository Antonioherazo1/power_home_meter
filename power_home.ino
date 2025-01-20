#include <ESP8266WiFi.h>
#include <WiFiManager.h>  // Incluye la librería WiFiManager
#include <PubSubClient.h>

// Configuración del broker MQTT
const char* mqtt_broker = "thinc.site";
const char* mqtt_topic = "energia/consumo";
const char* mqtt_username = "ad";
const char* mqtt_password = "mnea";
const int mqtt_port = 1883;

// Pines
const int sensorPin = A0; // Pin analógico para leer el sensor SCT-013-000

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void connectToMQTTBroker() {
    while (!mqtt_client.connected()) {
        String client_id = "esp8266-client-" + String(WiFi.macAddress());
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker");
        } else {
            Serial.print("Failed to connect to MQTT broker, rc=");
            Serial.print(mqtt_client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Inicia WiFiManager
    WiFiManager wifiManager;
    // Comenta esta línea si quieres que el dispositivo recuerde la red después de configurarla
    wifiManager.resetSettings();

    // Inicia el portal cautivo para configurar WiFi
    if (!wifiManager.autoConnect("Setup-WiFi", "password123")) {
        Serial.println("Failed to connect, restarting...");
        ESP.restart();
    }

    Serial.println("Connected to WiFi!");

    // Configura el cliente MQTT
    mqtt_client.setServer(mqtt_broker, mqtt_port);

    pinMode(sensorPin, INPUT);
}

void loop() {
    if (!mqtt_client.connected()) {
        connectToMQTTBroker();
    }
    mqtt_client.loop();

    // Leer el valor del sensor
    int rawValue = analogRead(sensorPin); // Lectura del ADC (0-1023)

    // Convertir el valor a voltaje
    float voltage = (rawValue / 1023.0) * 3.3; // Convertir a voltaje (3.3V referencia)

    // Calcular la corriente en amperios
    float current = (voltage / 1.0) * 100.0; // Según la relación 1V = 100A

    // Enviar el valor de corriente al broker MQTT
    mqtt_client.publish(mqtt_topic, String(current).c_str());

    // Mostrar el valor en el monitor serial
    Serial.print("Current (A): ");
    Serial.println(current, 2); // Mostrar con 2 decimales

    delay(1000); // Enviar datos cada segundo
}
