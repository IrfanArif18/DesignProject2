// Select your modem:
#define TINY_GSM_MODEM_SIM800

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG Serial

#include <TinyGsmClient.h>
#include <HardwareSerial.h>
#include <esp_now.h>  //library esp_now (udah ada dari sananya ga perlu download)
#include <WiFi.h> //library buat wifi (udah ada dari sananya ga perlu download)
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <NTPClient.h> // Library to get time of the day (hour, minute, and second)

int kapasitas = 50;
const int trigPin = 25;
const int echoPin = 33;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;

int upper_threshold = 100;
int lower_threshold = 30;
int sampling_time = 100;
int sensitivity = 1;

float t1;
float t2;
float t3;
float t4;

HardwareSerial SerialAT(2);
#define TX2 17
#define RX2 16
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

// Your GPRS credentials, if any
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// ========== Time conversion multipliers ==========
#define S_TO_uS 1000000
#define M_TO_S  60
#define H_TO_S  3600

// ========== Operational time variables ==========
int openHour = 6, closeHour = 22;
String formattedTime;
int currentHour, currentMinute, sleepHour = 0, sleepMinute = 0;
unsigned long sleepTime = 0;

// ========== Time modular function declarations ==========
void NTPClientInit();
void getCurrentTime();
void operationalSleep();

// MQTT details
//credential mqtt
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "esp32/message";
const char *topic_1 = "dev/upper_threshold";
const char *topic_3 = "dev/lower_threshold";
const char *topic_4 = "dev/sampling_time";
const char *topic_5 = "dev/sensitivity";
const char *topic_2 = "user/kapasitas";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;
char buffer [33];
PubSubClient mqttClient(client);    //biarin



//UNTUK ESP YG PINNYA ADA SEMUA

uint8_t broadcastAddress[] = {0x0C, 0xB8, 0x15, 0xF2, 0xE6, 0x1C};    //MAC address dari rx ;MAC address: buntung (tx) = 0C:B8:15:F2:D5:6C ; biasa = 0C:B8:15:F2:E6:1C ; kecil (rx) = 94:B9:7E:C2:CE:08

int incoming;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {           //data yg mau dikirim (pakai struct misalnya kalo banyak data yg dikirim)
    char a[32];                               
    int b;
    float c;
    bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;  //kasih nama ke struct

int masuk;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// fungsi yg dijalankan jika data di-receive
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  masuk = 0;
  memcpy(&myData, incomingData, sizeof(myData));
  
  //Serial.print("Bytes received: ");     //}
  //Serial.println(len);                  //}
  //Serial.print("Char: ");               //}
  //Serial.println(myData.a);             //}Kalo misalnya
  //Serial.print("Int: ");                //}mau pakai
  //Serial.println(myData.b);             //}variabel2 data
  //Serial.print("Float: ");              //}yg lain.
  //Serial.println(myData.c);             //}
  //Serial.print("Bool: ");               //}
  //Serial.println(myData.d);             //}

  Serial.println("Sensor masuk ter-trigger!");
  Serial.print("Kapasitas: ");
  Serial.println(kapasitas);
  mqttClient.publish(topic_2, itoa(kapasitas, buffer, 10 ));  //ngubah int jadi char soalnya bisanya publish char
  Serial.println();
  kapasitas -= myData.b;
  
}


 
void setup() {
  t1 = millis();
  // Initialize Serial Monitor        
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);    //biarin

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {                     //biarin
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);     //biarin

  //Start SIM800L Serial Communication
  SerialAT.begin(115200, SERIAL_8N1, RX2, TX2, false);
  delay(30);
  Serial.println("Initializing modem...");
  modem.restart();

  //modem & sim info
  String modemInfo = modem.getModemInfo();
  int simInfo = modem.getSimStatus();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);
  Serial.print("Sim Info: ");
  Serial.println(simInfo);

  
  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    delay(10000);
    return;
    }
  Serial.println(" success");
  

  if (modem.isNetworkConnected()){ 
    Serial.println("Network connected"); 
    }

  // GPRS connection parameters are usually set after network registration
  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" fail");
    delay(10000);
    return;
    }
  Serial.println(" success");

  if(modem.isGprsConnected()){
    Serial.println("GPRS connected"); 
    }

  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(callback);
  Serial.println("Connecting to MQTT...");
  while (!mqttClient.connected()) {
   String client_id = String(WiFi.macAddress());
  
     if (mqttClient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
         Serial.println("Connected to Public MQTT Broker");
         delay(5000);
         mqttClient.publish(topic_2, itoa(kapasitas, buffer, 10 ));
         Serial.print("Kapasitas: ");
         Serial.println(kapasitas);
     } else {
         Serial.println("Failed to connect with MQTT Broker");
         Serial.print(mqttClient.state());
         delay(2000);
     }
  }
  mqttClient.subscribe(topic); //biar bisa ngambil kiriman dari mqtt
  mqttClient.subscribe(topic_1);
  mqttClient.subscribe(topic_3);
  mqttClient.subscribe(topic_4);
  mqttClient.subscribe(topic_5);

  NTPClientInit();
}

//nampilin kiriman mqtt
void callback(char *topic, byte *payload, unsigned int length) {
  char mess[24];
 //Serial.print("Message arrived in topic: ");
 //Serial.println(topic);
 for (int i = 0; i < length; i++) {
     mess[i] = payload[i];
     //Serial.println("Payload to Mess Called");
 }
  toLow(mess);         //Make mess LowerCase

  if (strcmp(topic, topic_1) == 0){
    upper_threshold = atoi(mess);
    Serial.println("Upper Threshold: ");
    Serial.print(upper_threshold);
  }

  if (strcmp(topic, topic_3) == 0){
    lower_threshold = atoi(mess);
    Serial.println("Lower Threshold: ");
    Serial.print(lower_threshold);
  }

  if (strcmp(topic, topic_4) == 0){
    sampling_time = atoi(mess);
    Serial.println("Sampling Time: ");
    Serial.print(sampling_time);
  }

  if (strcmp(topic, topic_5) == 0){
    sensitivity = atoi(mess);
    Serial.println("Sensitivity: ");
    Serial.print(sensitivity);
  }
 
 Serial.println();
 Serial.println("************");
}

void toLow(char *p) {
  //Serial.println("Tolow called");
  while (*p) {
    *p = tolower(*p);
    p++;
  }
}
 
void loop() {

  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    Serial.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      Serial.println(" fail");
      delay(10000);
      return;
      }
    if (modem.isNetworkConnected()) {
      Serial.println("Network re-connected");
      }

    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      Serial.println("GPRS disconnected!");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) {
        Serial.println("GPRS reconnected"); 
        }
      }
    }

    while (!mqttClient.connected()) {
      String client_id = String(WiFi.macAddress());
      Serial.println("MQTT disconnected!");
       if (mqttClient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
           Serial.println("MQTT reconnected!");
           delay(3000);
       } 
       else {
           Serial.println("MQTT reconnection Failed!");
           Serial.print(mqttClient.state());
           delay(2000);
       }
   } 


  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;  

  Serial.print("Distance (cm): ");
  Serial.println(distanceCm); 


  
  if(distanceCm >= 5 && distanceCm <= 400) {   //jika jarak dibaca sensor diantara 20 - 80 cm
    if(kapasitas == 0) {
      kapasitas = 0;  //kayaknya boleh dihapus
      Serial.print("PARKIRAN PENUH!");
    }
    if(distanceCm >= lower_threshold && distanceCm <= upper_threshold) {  //Kapasitas ditambah jika jarak dibaca sensor dibawah 90 cm
      
      Serial.println();
      Serial.println("Sensor ter-trigger!");
      Serial.print("Distance (cm): ");
      Serial.println(distanceCm);
      kapasitas++;
      Serial.print("Kapasitas: ");
      Serial.println(kapasitas);
        //ngubah int jadi char soalnya bisanya publish char
      
      int jumlah_keluar = 1;    
      
      strcpy(myData.a, "THIS IS A CHAR");
      myData.b = jumlah_keluar;   //mengirim jumlah keluar (1)
      myData.c = jumlah_keluar;
      myData.d = false;

      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));   //mengirim data ke rx
   
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
      Serial.print("Waktu ter-trigger: ");
      Serial.print(millis()*0.001);
      Serial.println(" s");
      Serial.println();
    }
    //Serial.println();  
  }

  mqttClient.publish(topic_2, itoa(kapasitas, buffer, 10 ));

  operationalSleep();
  delay(1500);    //waktu sampling sensor
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
      sleepMinute = (59 - 15) - currentMinute; // Set wake up 15 minutes early
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
