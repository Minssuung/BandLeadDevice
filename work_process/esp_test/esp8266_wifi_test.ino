#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "PinkLAB";      // ← 본인 WiFi 이름
const char* password = "pinklab0523!";   // ← 본인 WiFi 비번
const char* pcIP     = "192.168.0.19";         // ← PC IP (지금 것 그대로)
const int   udpPort  = 4210;

WiFiUDP udp;
int count = 0;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi 연결 완료! IP: " + WiFi.localIP().toString());
}

void loop() {
  udp.beginPacket(pcIP, udpPort);
  String msg = "Hello from ESP8266! count=" + String(count++);
  udp.write(msg.c_str());
  udp.endPacket();
  Serial.println("전송: " + msg);
  delay(1000);
}