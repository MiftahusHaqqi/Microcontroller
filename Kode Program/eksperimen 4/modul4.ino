// Modul 4

#include <WiFi.h>
#include <PubSubClient.h>

// ================= WIFI =================
const char* ssid = "TAHU";
const char* password = "emokmales123";

// ================= MQTT =================
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttTopic = "kontrolbas";
const char* mqttTopicSpeed = "kontrolbas/speed";

// ================= PIN ==================
int ledPin = 2;          // LED onboard
int IN1 = 27;            // Driver IN1
int IN2 = 26;            // Driver IN2
int ENA = 12;            // Driver ENA (PWM)

// ================= PWM ==================
const int pwmChannel = 0;
const int pwmFreq = 30000;
const int pwmResolution = 8;
int duty = 0;

// ================= MQTT CLIENT ==========
WiFiClient espClient;
PubSubClient client(espClient);

// =======================================

void connectWiFi() {
  Serial.print("Connecting WiFi ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi CONNECTED");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting MQTT...");
    String clientID = "ESP32-" + String(random(0xffff), HEX);

    if (client.connect(clientID.c_str())) {
      Serial.println("CONNECTED");
      client.subscribe(mqttTopic);
      client.subscribe(mqttTopicSpeed);
      Serial.print("Subscribe: ");
      Serial.println(mqttTopic);
    } else {
      Serial.print("FAILED, rc=");
      Serial.print(client.state());
      delay(3000);
    }
  }
}

// ================= MOTOR =================
void motorON() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // soft start
  for (int d = 0; d <= 200; d += 20) {
    ledcWrite(pwmChannel, d);
    delay(100);
  }

  digitalWrite(ledPin, HIGH);
  Serial.println("MOTOR ON");
}

void motorOFF() {
  for (int d = 200; d >= 0; d -= 20) {
    ledcWrite(pwmChannel, d);
    delay(50);
  }

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ledPin, LOW);

  Serial.println("MOTOR OFF");
}

// =============== MQTT CALLBACK ===========
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("Topic: ");
  Serial.print(topic);
  Serial.print(" | Payload: ");
  Serial.println(msg);

  // ===== ON / OFF =====
  if (String(topic) == "kontrolbas") {
    if (msg == "1" || msg == "ON") {
      motorON();
    }
    if (msg == "0" || msg == "OFF") {
      motorOFF();
    }
  }

  // ===== SPEED SLIDER =====
  if (String(topic) == "kontrolbas/speed") {
    duty = msg.toInt();
    duty = constrain(duty, 0, 255);
    ledcWrite(pwmChannel, duty);

    Serial.print("Speed set: ");
    Serial.println(duty);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(ENA, pwmChannel);
  ledcWrite(pwmChannel, 0);

  connectWiFi();

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

// ================= LOOP ==================
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
}
