// ===== Libraries =====
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// ===== Timer Parameters =====
#define S_TO_uS 1000000
#define M_TO_S  60
#define H_TO_S  3600

// ===== WiFi Connections and NTP Client =====
const char* WIFI_SSID = "SSID";
const char* WIFI_PASS = "password";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// ===== Timer Variables =====
int openHour = 6, closeHour = 22;

String formattedTime;
int currentHour, currentMinute, sleepHour = 0, sleepMinute = 0;
unsigned long sleepTime = 0;

// ===== Modular Functions Declaration =====
void connectWiFi();
void NTPClientInit();

void getCurrentTime();
void operationalSleep();

// ===== Setup =====
void setup() {
  delay(5000);
  Serial.begin(9600);
  
  connectWiFi();
  NTPClientInit();
}

// ===== Loop =====
void loop() {
  operationalSleep();
  delay(5000);
}

// ===== Modular Functions Definition =====
void connectWiFi() {
  long startTime, loopTime;
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(". ");
    delay(500);
    loopTime = millis() - startTime;

    if (loopTime > 9000) {
      Serial.println("");
      Serial.println("Connection failed");
      delay(1000);
    }
  }

  Serial.println("");
  Serial.println("Connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
}

void NTPClientInit() {
  timeClient.begin();
  timeClient.setTimeOffset(7 * 3600); // GMT + 7
}

void getCurrentTime() {
  timeClient.update();
  formattedTime = timeClient.getFormattedTime();
  currentHour = timeClient.getHours();
  currentMinute = timeClient.getMinutes();
}

void operationalSleep() {
  getCurrentTime();

  // If system is in operating hours
  if (currentHour >= openHour && currentHour < closeHour) {
    return;
  }

  // Otherwise if closing hours
  Serial.print("Currently, it's ");
  Serial.println(formattedTime);

  if (currentHour >= closeHour && currentHour < 24) {
    sleepHour = 24 - currentHour + openHour;
  } else if (currentHour < openHour) {
    sleepHour = openHour - currentHour;
  }

  Serial.print("Next operating hour is at ");
  Serial.print(openHour);
  Serial.println(":00");

  if (currentMinute > 0) {
      sleepHour -= 1;
      sleepMinute = 59 - currentMinute;
  }

  sleepTime = sleepHour * H_TO_S;
  sleepTime += sleepMinute * M_TO_S;
  sleepTime = sleepTime * S_TO_uS;

  if (sleepTime > 0) {
    Serial.print("Going to sleep for ");
    Serial.print(sleepHour);
    Serial.print(" hour(s) and ");
    Serial.print(sleepMinute);
    Serial.println(" minute(s)");
    Serial.println("==================================================");
    esp_sleep_enable_timer_wakeup(sleepTime);
    Serial.flush();
    delay(3000);
    esp_deep_sleep_start();
  }
}
