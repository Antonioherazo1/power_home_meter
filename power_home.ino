#include <ESP8266WiFi.h>      // Incluye la biblioteca para manejar conexiones Wi-Fi con el ESP8266.
#include <WiFiManager.h>      // Biblioteca para facilitar la configuración de Wi-Fi mediante un portal cautivo.
#include <PubSubClient.h>     // Biblioteca para manejar el protocolo MQTT, que se utiliza para comunicación de mensajes entre dispositivos.

// Configuración del broker MQTT
const char* mqtt_broker = "thinc.site";   // Dirección del servidor MQTT.
const char* mqtt_topic = "energia/consumo"; // Tópico MQTT donde se publicarán los datos (consumo de energía en este caso).
const char* mqtt_username = "ad";         // Nombre de usuario para autenticar en el broker MQTT.
const char* mqtt_password = "mnea";       // Contraseña para autenticar en el broker MQTT.
const int mqtt_port = 1883;               // Puerto para la conexión MQTT (1883 es el puerto estándar).

// Pines
const int sensorPin = A0;                 // Pin analógico del ESP8266 utilizado para leer el sensor SCT-013-000.

// Objetos para manejar Wi-Fi y MQTT
WiFiClient espClient;                     // Cliente Wi-Fi para la conexión a Internet.
PubSubClient mqtt_client(espClient);      // Cliente MQTT que usa `espClient` para manejar las comunicaciones.

/// @brief Función para conectar al broker MQTT.
void connectToMQTTBroker() {
    // Mientras no esté conectado al broker...
    while (!mqtt_client.connected()) {
        // Generar un ID de cliente único utilizando la dirección MAC del ESP8266.
        String client_id = "esp8266-client-" + String(WiFi.macAddress());
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        
        // Intentar conectar al broker con el nombre de usuario y contraseña.
        if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker"); // Mensaje si la conexión es exitosa.
        } else {
            // Mensaje si la conexión falla, mostrando el código de error.
            Serial.print("Failed to connect to MQTT broker, rc=");
            Serial.print(mqtt_client.state()); // Código de estado de la conexión MQTT.
            Serial.println(" try again in 5 seconds");
            delay(5000); // Esperar 5 segundos antes de volver a intentar.
        }
    }
}

/// @brief Configuración inicial del ESP8266.
void setup() {
    Serial.begin(115200); // Inicializa la comunicación serial a 115200 baudios para monitoreo.

    // Crear un objeto WiFiManager para manejar la conexión Wi-Fi.
    WiFiManager wifiManager;
    // Restablecer la configuración de Wi-Fi previamente guardada (opcional, solo si es necesario).
    wifiManager.resetSettings();

    // Inicia el portal cautivo para configurar la red Wi-Fi.
    if (!wifiManager.autoConnect("Setup-WiFi", "password123")) {
        // Si no se logra conectar al Wi-Fi, reiniciar el ESP8266.
        Serial.println("Failed to connect, restarting...");
        ESP.restart();
    }

    Serial.println("Connected to WiFi!"); // Confirmación de que el ESP8266 se conectó a la red Wi-Fi.

    // Configurar el cliente MQTT con la dirección del broker y el puerto.
    mqtt_client.setServer(mqtt_broker, mqtt_port);

    // Configurar el pin del sensor como entrada.
    pinMode(sensorPin, INPUT);
}

/// @brief Bucle principal del ESP8266.
void loop() {
    // Si el cliente MQTT no está conectado, intentar reconectar.
    if (!mqtt_client.connected()) {
        connectToMQTTBroker();
    }
    mqtt_client.loop(); // Mantener la conexión MQTT activa.

    // Leer el valor analógico del pin conectado al sensor.
    int rawValue = analogRead(sensorPin); // Lectura del ADC (valor entre 0 y 1023).

    // Convertir la lectura ADC a voltaje.
    float voltage = (rawValue / 1023.0) * 3.3; // Convertir a un valor de voltaje basado en la referencia de 3.3V.

    // Calcular la corriente en amperios según la salida del sensor.
    float current = (voltage / 1.0) * 100.0; // SCT-013 tiene una relación de 1V = 100A.

    // Publicar el valor calculado de corriente en el tópico MQTT configurado.
    mqtt_client.publish(mqtt_topic, String(current).c_str());

    // Mostrar el valor de corriente en el monitor serial para depuración.
    Serial.print("Current (A): ");
    Serial.println(current, 2); // Imprimir con dos decimales.

    delay(1000); // Esperar 1 segundo antes de realizar otra lectura y publicación.
}
