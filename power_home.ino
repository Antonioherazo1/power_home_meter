#include <ESP8266WiFi.h>      // Biblioteca para manejar la conexión Wi-Fi con el ESP8266.
#include <PubSubClient.h>     // Biblioteca para manejar la comunicación MQTT (publicar y suscribirse a tópicos).
#include <WiFiManager.h>      // Biblioteca que facilita la configuración Wi-Fi mediante un portal cautivo.
#include <Arduino.h>          // Biblioteca principal de Arduino para utilizar las funciones básicas del sistema.


const int currentPin = A0;  // Pin para medir la corriente (SCT-013)
const float sensitivity = 0.1;  // Sensibilidad del SCT-013 (100A/1V = 0.1 V/A)
const float vRef = 3.3;  // Voltaje de referencia del ESP8266 (3.3V)
const int adcResolution = 1023;  // Resolución del ADC (10 bits)
const float fixedVoltage = 220.0;  // Voltaje fijo (220V o 110V)
const float powerFactor = 1.0;  // Factor de potencia (asume 1 si no se conoce)
const int numSamples = 1000;  // Número de muestras para calcular RMS
const float acFrequency = 50.0;  // Frecuencia de la red eléctrica (50 Hz o 60 Hz)

// Configuración del sensor SCT-013-000
const int sensorPin = A0;               // Pin analógico al que está conectado el sensor SCT-013-000.
const float sensorFactor = 100.0;       // Relación del sensor: 1V = 100A, utilizada para convertir el voltaje a corriente.

// Configuración del broker MQTT
const char* mqtt_broker = "thinc.site"; // Dirección del servidor MQTT.
const char* mqtt_username = "ad";       // Nombre de usuario para autenticar la conexión MQTT.
const char* mqtt_password = "mnea";     // Contraseña para autenticar la conexión MQTT.
const int mqtt_port = 1883;             // Puerto estándar para MQTT.
const char* mqtt_topic_current = "consumo/amps"; // Tópico MQTT donde se publicará el valor de la corriente.
const char* mqtt_topic_reset = "consumo/reset";   // Tópico MQTT para recibir comandos de reinicio del Wi-Fi.

// Objetos para manejar conexiones Wi-Fi y MQTT
WiFiClient espClient;                   // Cliente Wi-Fi para conectarse a redes.
PubSubClient mqtt_client(espClient);    // Cliente MQTT que utiliza `espClient`.
WiFiManager wifiManager;                // Objeto WiFiManager para manejar la configuración Wi-Fi.

/// @brief Conecta el ESP8266 a una red Wi-Fi utilizando WiFiManager.
void connectToWiFi() {
    Serial.println("Configurando WiFi...");
    // Inicia el portal cautivo para la configuración Wi-Fi. 
    // Si no se puede conectar, el dispositivo se reinicia.
    if (!wifiManager.autoConnect("Setup-WiFi", "password123")) {
        Serial.println("No se pudo conectar, reiniciando...");
        ESP.restart();
    }
    Serial.println("Conectado al WiFi."); // Confirmación de conexión.
}

/// @brief Conecta al broker MQTT y se suscribe a los tópicos necesarios.
void connectToMQTTBroker() {
    // Mientras no esté conectado al broker MQTT...
    while (!mqtt_client.connected()) {
        // Generar un ID único para el cliente MQTT basado en la dirección MAC del ESP8266.
        String client_id = "esp8266-client-" + String(WiFi.macAddress());
        Serial.printf("Conectando al broker MQTT como %s...\n", client_id.c_str());
        // Intentar conectar al broker con las credenciales proporcionadas.
        if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Conectado al broker MQTT.");
            mqtt_client.subscribe(mqtt_topic_reset); // Suscribirse al tópico que controla el reinicio del Wi-Fi.
        } else {
            // Mostrar mensaje de error si la conexión falla e intenta reconectar después de 5 segundos.
            Serial.println("Error al conectar al broker. Reintentando en 5 segundos...");
            delay(5000);
        }
    }
}

/// @brief Callback que procesa los mensajes recibidos en los tópicos suscritos.
/// @param topic Tópico del mensaje recibido.
/// @param payload Cuerpo del mensaje recibido.
/// @param length Longitud del mensaje.
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message = ""; // Almacena el mensaje recibido como una cadena.
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i]; // Construye el mensaje carácter por carácter.
    }

    // Verifica si el mensaje pertenece al tópico de reinicio y ejecuta el reinicio del Wi-Fi.
    if (String(topic) == mqtt_topic_reset && message == "RESET") {
        Serial.println("¡Reset WiFi recibido por MQTT!");
        wifiManager.resetSettings(); // Borra las credenciales Wi-Fi almacenadas.
        ESP.restart();               // Reinicia el ESP8266.
    }
}

/// @brief Lee el valor del sensor SCT-013-000 y lo convierte a corriente.
/// @return Corriente medida en amperios.
float readCurrent() {
    int rawValue = analogRead(sensorPin);       // Lee el valor analógico del pin conectado al sensor.
    float voltage = (rawValue / 1023.0) * 3.3; // Convierte el valor ADC a voltaje (referencia de 3.3V).
    float current = voltage * sensorFactor;    // Convierte el voltaje a corriente utilizando el factor del sensor.
    return current;
}

/// @brief Configuración inicial del ESP8266.
void setup() {
    Serial.begin(115200); // Inicia la comunicación serial a 115200 baudios para monitoreo.

    pinMode(sensorPin, INPUT); // Configura el pin del sensor como entrada.
    connectToWiFi();           // Conecta a la red Wi-Fi utilizando WiFiManager.

    // Configura el cliente MQTT con el servidor y el puerto del broker.
    mqtt_client.setServer(mqtt_broker, mqtt_port);
    mqtt_client.setCallback(mqttCallback); // Define la función que procesará los mensajes MQTT.
    connectToMQTTBroker();                // Establece la conexión inicial con el broker MQTT.
}

/// @brief Bucle principal del ESP8266.
void loop() {
    // Verifica si el cliente MQTT sigue conectado; de lo contrario, intenta reconectar.
    if (!mqtt_client.connected()) {
        connectToMQTTBroker();
    }
    mqtt_client.loop(); // Mantiene la conexión MQTT y procesa los mensajes entrantes.

    // Calculo corriente y potencia. 
    float currentRMS = calculateRMS(currentPin, sensitivity);
    float power = fixedVoltage * currentRMS * powerFactor;  // Potencia en vatios
    
    // Crear un mensaje con el valor de corriente en formato de texto con dos decimales.
    char currentMessage[50];
    snprintf(currentMessage, sizeof(currentMessage), "%.2f", currentRMS);
    
    // Publicar el valor de corriente en el tópico MQTT correspondiente.
    mqtt_client.publish(mqtt_topic_current, currentMessage);

    // Mostrar el valor de corriente en el monitor serial para depuración.
    Serial.print("Current RMS: ");
    Serial.print(currentRMS);
    Serial.print(" A  Power: ");
    Serial.print(power);
    Serial.println(" W");

    delay(1000); // Esperar 1 segundo antes de realizar la próxima lectura y publicación.
}


float calculateRMS(int pin, float calibration) {
  float sumSquares = 0;
  for (int i = 0; i < numSamples; i++) {
    float adcValue = readCurrent();
    float voltage = (adcValue / (float)adcResolution) * vRef;
    float value = (voltage - (vRef / 2)) * calibration;  // Eliminar el offset (1.65V)
    sumSquares += value * value;
    delayMicroseconds(1000000 / (acFrequency * numSamples));  // Muestreo sincronizado
  }
  return sqrt(sumSquares / numSamples);
}