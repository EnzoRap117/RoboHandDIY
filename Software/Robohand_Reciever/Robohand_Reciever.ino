/*  This program requires a another board wired up to a lucidGlove
    running "lucidgloves-firmware_custom.ino"

    Refer to "One_Way_Comm_Test_RecieverForGlove" for more explanation
    as to how the data tranmission works
*/
#include <esp_now.h>
#include <WiFi.h>
#include "ESP32Servo.h"

 // create Servo object to control a servo
Servo pinkyServo;
Servo ringServo;
Servo middleServo;
Servo indexServo;
Servo thumbServo;
Servo wristServo;
Servo forearmServo;

Servo myServos[] = {thumbServo, indexServo, middleServo, ringServo, pinkyServo};
int len = 5;

//int potValues[5];
char letters[] = {'A', 'B', 'C', 'D', 'E'};

typedef struct struct_message {
    //char a[32];
    byte thumbVal;
    byte indexVal;
    byte middleVal;
    byte ringVal;
    byte pinkyVal;
    byte wristVal;
    byte forearmVal;
    //int GloveData[5];
} struct_message;

struct_message myData;

int val;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  /*
  for(int k = 0; k < 5; k++){
    Serial.print(letters[k]);
    Serial.print(": ");
    Serial.print(myData.GloveData[k]);
    Serial.print(" ");
  }
  Serial.println();
  */

  /*
  for(int i = 0; i < 5; i++){
    val = myData.GloveData[i];
    val = map(val, 0, 4095, 0, 180);
    Serial.print(letters[i]);
    Serial.print(": ");
    Serial.print(val);
    Serial.print(" ");
  }
  */
  Serial.println();
  moveServos();
}

void moveServos(){

  
  thumbServo.write(myData.thumbVal);

  indexServo.write(myData.indexVal);

  middleServo.write(myData.middleVal);

  ringServo.write(myData.ringVal);
 
  pinkyServo.write(myData.ringVal);

  wristServo.write(myData.wristVal);

  forearmServo.write(myData.forearmVal);
  
}

void setup() {
  pinkyServo.attach(19); //27
  ringServo.attach(18); //5
  middleServo.attach(5); //18
  indexServo.attach(17); //19
  thumbServo.attach(21);
  wristServo.attach(5);
  foreServo.attach(16);

  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {

}

  