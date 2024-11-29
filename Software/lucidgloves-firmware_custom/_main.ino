//#define comm ICommunication*;

#define ALWAYS_CALIBRATING CALIBRATION_LOOPS == -1

#define CALIB_OVERRIDE true
#if USING_CALIB_PIN && COMMUNICATION == COMM_SERIAL && PIN_CALIB == 0 && !CALIB_OVERRIDE
  #error "You can't set your calibration pin to 0 over usb. You can calibrate with the BOOT button when using bluetooth only. Set CalibOverride to true to override this."
#endif

//ESPNOW Necessary Items
  #include <esp_now.h>
  #include <WiFi.h>

  //Sheild address/MAC Address. Replace the address below with the one from the robot's ESP32 follwing the below format.
  uint8_t broadcastAddress[] = {0x44, 0xbc, 0xa2, 0x22, 0xa1, 0x80};

  // Structure example to send data
  // Must match the receiver structure
  typedef struct struct_message {
    //char a[32];
    byte thumbVal;
    byte indexVal;
    byte middleVal;
    byte ringVal;
    byte pinkyVal;
    byte wristVal;
    byte forearmVal;
    //int GloveArray[5];
  } struct_message;

  // Create a struct_message called myData
  struct_message myData;
 // int myGloveArray[5];
 // int* glovePtr = myGloveArray;

  //int myGloveArray[] = {4095, 4095, 4095, 4095, 4095};

  esp_now_peer_info_t peerInfo;


  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }


// --- ESPNOW Setup end ---

ICommunication* comm;
int loops = 0;
void setup() {
  //EPSNOW
  // Init Serial Monitor
  Serial.begin(115200);

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

  //LUCID Code
  #if COMMUNICATION == COMM_SERIAL
    comm = new SerialCommunication();
  #elif COMMUNICATION == COMM_BTSERIAL
    comm = new BTSerialCommunication();
  #endif  
  comm->start();

  setupInputs();

  #if USING_FORCE_FEEDBACK
    setupServoHaptics();  
  #endif
  
}

void loop() {
  Serial.print("\n"); 

  if (comm->isOpen()){
    #if USING_CALIB_PIN
    bool calibButton = getButton(PIN_CALIB) != INVERT_CALIB;
    if (calibButton)
      loops = 0;
    #else
    bool calibButton = false;
    #endif
    
    bool calibrate = false;
    if (loops < CALIBRATION_LOOPS || ALWAYS_CALIBRATING){
      calibrate = true;
      loops++;
    }
    
    int* fingerPos = getFingerPositions(calibrate, calibButton);

    int fingerPose[5];
    memcpy(fingerPose, fingerPos, sizeof(fingerPose));
    myData.thumbVal = map(fingerPose[0], 0, 4095, 0, 180);
    myData.indexVal = map(fingerPose[1], 0, 4095, 0, 180);
    myData.middleVal = map(fingerPose[2], 0, 4095, 0, 180);
    myData.ringVal = map(fingerPose[3], 0, 4095, 0, 180);
    myData.pinkyVal = map(fingerPose[4], 0, 4095, 0, 180);
    myData.wristVal = map(analogRead(PIN_WRIST), 0, 4095, 0, 180);
    myData.foreVal = map(analogRead(PIN_FOREARM), 0, 4095, 0, 180);
    //memcpy(myData.GloveArray, fingerPose, sizeof(myData.GloveArray));
    
    

    bool joyButton = getButton(PIN_JOY_BTN) != INVERT_JOY;

    #if TRIGGER_GESTURE
    bool triggerButton = triggerGesture(fingerPos);
    #else
    bool triggerButton = getButton(PIN_TRIG_BTN) != INVERT_TRIGGER;
    #endif

    bool aButton = getButton(PIN_A_BTN) != INVERT_A;
    bool bButton = getButton(PIN_B_BTN) != INVERT_B;

    #if GRAB_GESTURE
    bool grabButton = grabGesture(fingerPos);
    #else
    bool grabButton = getButton(PIN_GRAB_BTN) != INVERT_GRAB;
    #endif

    #if PINCH_GESTURE
    bool pinchButton = pinchGesture(fingerPos);
    #else
    bool pinchButton = getButton(PIN_PNCH_BTN) != INVERT_PINCH;
    #endif

    bool menuButton = getButton(PIN_MENU_BTN) != INVERT_MENU;


    //Atempting to send this ^^
    //strcpy(myData.a, "LucidGlove data");
    //Serial.println(encode(fingerPos, getJoyX(), getJoyY(), joyButton, triggerButton, aButton, bButton, grabButton, pinchButton, calibButton, menuButton));
    //encode(myGloveArray, fingerPos, getJoyX(), getJoyY(), joyButton, triggerButton, aButton, bButton, grabButton, pinchButton, calibButton, menuButton);
    //myData.GloveArray = myGloveArray;
    //myGloveArray = getFingerData(encode(fingerPos, getJoyX(), getJoyY(), joyButton, triggerButton, aButton, bButton, grabButton, pinchButton, calibButton, menuButton));
    //memcpy(myData.GloveArray, getFingerData(encode(fingerPos, getJoyX(), getJoyY(), joyButton, triggerButton, aButton, bButton, grabButton, pinchButton, calibButton, menuButton)), sizeof(myData.GloveArray));

    
    //int* dataPtr = getFingerData(encode(fingerPos, getJoyX(), getJoyY(), joyButton, triggerButton, aButton, bButton, grabButton, pinchButton, calibButton, menuButton));
    
    //for (int i = 0; i < 5; i++) {
    //*(glovePtr + i) = *(dataPtr + i);
    //}
    
    //memcpy(myData.GloveArray, myGloveArray, sizeof(myData.GloveArray));
  

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    
    if (result == ESP_OK) {
    Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    
    
    // --End of Send--
    
    comm->output(encode(fingerPos, getJoyX(), getJoyY(), joyButton, triggerButton, aButton, bButton, grabButton, pinchButton, calibButton, menuButton));

    #if USING_FORCE_FEEDBACK
      char received[100];
      if (comm->readData(received)){
        int hapticLimits[5];
        //This check is a temporary hack to fix an issue with haptics on v0.5 of the driver, will make it more snobby code later
        if(String(received).length() >= 10) {
           decodeData(received, hapticLimits);
           writeServoHaptics(hapticLimits); 
        }
      }
    #endif
    //delay(LOOP_TIME);
    Serial.println("MAIN LOOP COMPLETE");
    delay(2);
  }
}
