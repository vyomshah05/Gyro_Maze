#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include <ESP32Servo.h>     // Use ESP32Servo library on ESP32
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "DHT20.h"
#include <ArduinoJson.h>
#include <string>
// Wi-Fi credentials
#define WIFI_SSID "Vyomphone"         // NOTE: Please delete this value before submitting assignment
#define WIFI_PASSWORD "48174817" // NOTE: Please delete this value before submitting assignment
// Azure IoT Hub configuration
#define SAS_TOKEN "SharedAccessSignature sr=cs147group.azure-devices.net%2Fdevices%2F147esp32&sig=Y935uKZ35%2BVJWWTOP10q%2FVPz1GCQfbzWYDp%2FbgABnjQ%3D&se=2356418040"
const char *root_ca = "-----BEGIN CERTIFICATE-----\n"\
"MIIEtjCCA56gAwIBAgIQCv1eRG9c89YADp5Gwibf9jANBgkqhkiG9w0BAQsFADBh\n"\
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"\
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"\
"MjAeFw0yMjA0MjgwMDAwMDBaFw0zMjA0MjcyMzU5NTlaMEcxCzAJBgNVBAYTAlVT\n"\
"MR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xGDAWBgNVBAMTD01TRlQg\n"\
"UlMyNTYgQ0EtMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMiJV34o\n"\
"eVNHI0mZGh1Rj9mdde3zSY7IhQNqAmRaTzOeRye8QsfhYFXSiMW25JddlcqaqGJ9\n"\
"GEMcJPWBIBIEdNVYl1bB5KQOl+3m68p59Pu7npC74lJRY8F+p8PLKZAJjSkDD9Ex\n"\
"mjHBlPcRrasgflPom3D0XB++nB1y+WLn+cB7DWLoj6qZSUDyWwnEDkkjfKee6ybx\n"\
"SAXq7oORPe9o2BKfgi7dTKlOd7eKhotw96yIgMx7yigE3Q3ARS8m+BOFZ/mx150g\n"\
"dKFfMcDNvSkCpxjVWnk//icrrmmEsn2xJbEuDCvtoSNvGIuCXxqhTM352HGfO2JK\n"\
"AF/Kjf5OrPn2QpECAwEAAaOCAYIwggF+MBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYD\n"\
"VR0OBBYEFAyBfpQ5X8d3on8XFnk46DWWjn+UMB8GA1UdIwQYMBaAFE4iVCAYlebj\n"\
"buYP+vq5Eu0GF485MA4GA1UdDwEB/wQEAwIBhjAdBgNVHSUEFjAUBggrBgEFBQcD\n"\
"AQYIKwYBBQUHAwIwdgYIKwYBBQUHAQEEajBoMCQGCCsGAQUFBzABhhhodHRwOi8v\n"\
"b2NzcC5kaWdpY2VydC5jb20wQAYIKwYBBQUHMAKGNGh0dHA6Ly9jYWNlcnRzLmRp\n"\
"Z2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcnQwQgYDVR0fBDswOTA3\n"\
"oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0R2xvYmFsUm9v\n"\
"dEcyLmNybDA9BgNVHSAENjA0MAsGCWCGSAGG/WwCATAHBgVngQwBATAIBgZngQwB\n"\
"AgEwCAYGZ4EMAQICMAgGBmeBDAECAzANBgkqhkiG9w0BAQsFAAOCAQEAdYWmf+AB\n"\
"klEQShTbhGPQmH1c9BfnEgUFMJsNpzo9dvRj1Uek+L9WfI3kBQn97oUtf25BQsfc\n"\
"kIIvTlE3WhA2Cg2yWLTVjH0Ny03dGsqoFYIypnuAwhOWUPHAu++vaUMcPUTUpQCb\n"\
"eC1h4YW4CCSTYN37D2Q555wxnni0elPj9O0pymWS8gZnsfoKjvoYi/qDPZw1/TSR\n"\
"penOgI6XjmlmPLBrk4LIw7P7PPg4uXUpCzzeybvARG/NIIkFv1eRYIbDF+bIkZbJ\n"\
"QFdB9BjjlA4ukAg2YkOyCiB8eXTBi2APaceh3+uBLIgLk8ysy52g2U3gP7Q26Jlg\n"\
"q/xKzj3O9hFh/g==\n"\
"-----END CERTIFICATE-----";
String iothubName = "cs147group"; 
String deviceName = "147esp32";     // Your device name (replace if needed)
String url = "https://" + iothubName + ".azure-devices.net/devices/" +
             deviceName + "/messages/events?api-version=2021-04-12";
// Telemetry interval
#define TELEMETRY_INTERVAL 3000 // Send data every 3 seconds
DHT20 dht;
uint8_t count = 0;
uint32_t lastTelemetryTime = 0;

LSM6DSO myIMU; // Default I2C, addr 0x6B

// Servo pins
static const int SERVO_X_PIN = 26;     // X-axis servo
static const int SERVO_Y_PIN = 27;     // Y-axis servo
const uint32_t DEBOUNCE_MS = 40;

const int SENSOR_PIN = 38;    
const bool ACTIVE_HIGH = true;        

bool lastStable = false;
bool lastRead     = false;
uint32_t lastChange = 0;
uint32_t startTime = 0;

bool readBallDetected() {
    int v = digitalRead(SENSOR_PIN);
    return ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}

Servo servoX, servoY;

void setup() {
    Serial.begin(9600);
    delay(1000);
    Wire.begin();
    WiFi.mode(WIFI_STA);
    dht.begin();
    delay(1000);
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);
    // Initialize DHT20 here
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        Serial.print(WiFi.status());
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("MAC address: ");
    Serial.println(WiFi.macAddress());
    // I2C (TTGO T-Display: SDA=21, SCL=22)
    Wire.begin(21, 22);
    delay(10);
    startTime = millis();
    // IMU init
    if (myIMU.begin())
    Serial.println("IMU Ready.");
    else {
    Serial.println("Could not connect to IMU. Freezing.");
    while (1);
    }

    if (myIMU.initialize(BASIC_SETTINGS))
    Serial.println("Loaded IMU Settings.");

    // Servos
    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);
    servoX.write(90);
    servoY.write(90);
    Serial.println("Servos attached.");

    pinMode(SENSOR_PIN, INPUT);         // module drives the line; no pullups needed
    Serial.println("TCRT5000 detector ready...");

}

void loop() {
    // Read tilt in g
    float ax = myIMU.readFloatAccelX();    // roll
    float ay = myIMU.readFloatAccelY();    // pitch

    bool curr = readBallDetected();

    if (curr != lastRead) {             // input changed
    lastRead = curr;
    lastChange = millis();
    }

    // Map -1g..+1g -> 0..180Â°
    int xAngle = map((int)(ax * 1000), -1000, 1000, 0, 180);
    int yAngle = map((int)(ay * 1000), -1000, 1000, 0, 180);

    // Optional invert for natural feel (flip if motion feels backward)
    // yAngle = 180 - yAngle;

    // Clamp & tiny deadband around center to reduce chatter
    xAngle = (90 + ((xAngle-90)*0.4));
    yAngle = 90 + ((yAngle-90)*0.4);
    xAngle = constrain(xAngle, 0, 180);
    xAngle = 180 - xAngle;
    yAngle = constrain(yAngle, 0, 180);
    const int center = 90, db = 2;
    if (abs(xAngle - center) <= db) xAngle = center;
    if (abs(yAngle - center) <= db) yAngle = center;

    // Drive servos

    servoX.write(xAngle);
    servoY.write(yAngle);

    // Debug
    //Serial.print("AccelX: "); Serial.print(ax, 3);
    //Serial.print(" g -> X servo: "); Serial.print(xAngle);
    //Serial.print("     |     AccelY: "); Serial.print(ay, 3);
    //Serial.print(" g -> Y servo: "); Serial.println(yAngle);
    if ((millis() - lastChange) > DEBOUNCE_MS && curr != lastStable) {
    lastStable = curr;
    if (lastStable){ 
        Serial.println("BALL DETECTED");
        uint32_t time_score = millis()-startTime;
        Serial.println(millis()-startTime);

        ArduinoJson::JsonDocument doc;
        doc["timeMs"]  = time_score;
        doc["minutes"] = (time_score/1000)/60;
        doc["seconds"] = (time_score/1000)%60;
        char buffer[256];
        serializeJson(doc, buffer, sizeof(buffer));
        // Send telemetry via HTTPS
        WiFiClientSecure client;
        client.setCACert(root_ca); // Set root CA certificate
        HTTPClient http;
        http.begin(client, url);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Authorization", SAS_TOKEN);
        int httpCode = http.POST(buffer);
        if (httpCode == 204)
        { // IoT Hub returns 204 No Content for successful telemetry
            Serial.println("Telemetry sent: " + String(buffer));
        }
        else
        {
            Serial.println("Failed to send telemetry. HTTP code: " + String(httpCode));
        }
        http.end();

    }
    else            Serial.println("No ball.");
    }
    delay(50);
}