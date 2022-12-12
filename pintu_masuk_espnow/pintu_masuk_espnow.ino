#include <esp_now.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h> // Library to get time of the day (hour, minute, and second)

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xC2, 0xCE, 0x08};    //MAC address dari rx ;MAC address: buntung (tx) = 0C:B8:15:F2:D5:6C ; biasa = 0C:B8:15:F2:E6:1C ; kecil (rx) = 94:B9:7E:C2:CE:08

const int trigPin = 23;
const int echoPin = 22;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;
                                          
// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {           //data yg mau dikirim (pakai struct misalnya kalo banyak data yg dikirim)
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;   //kasih nama ke struct                                     

esp_now_peer_info_t peerInfo;

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

// callback when data is sent (konfirmasi jika data berhasil dikirim)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  
  memcpy(&myData, incomingData, sizeof(myData));
  
  //Serial.print("Bytes received: ");     //}
  //Serial.println(len);                  //}
  //Serial.print("Char: ");               //}
  //Serial.println(myData.a);             //}Kalo misalnya
  Serial.print("Int: ");                //}mau pakai
  Serial.println(myData.b);             //}variabel2 data
  //Serial.print("Float: ");              //}yg lain.
  //Serial.println(myData.c);             //}
  //Serial.print("Bool: ");               //}
  //Serial.println(myData.d);             //}

  Serial2.write(myData.b);

  Serial.println("Sensor keluar ter-trigger!");
  Serial.println();
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
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

  esp_now_register_recv_cb(OnDataRecv);

  NTPClientInit();
}
 
void loop() {
  
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
    if(distanceCm <= 100) {  //Kapasitas ditambah jika jarak dibaca sensor dibawah 90 cm
      
      Serial.println();
      Serial.println("Sensor ter-trigger!");
      Serial.print("Distance (cm): ");
      Serial.println(distanceCm);     
      
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

  operationalSleep();
  delay(500);    //waktu sampling sensor
  
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
