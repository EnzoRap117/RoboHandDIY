/*
 * LucidGloves Firmware Version 4
 * Author: Lucas_VRTech - LucidVR
 * lucidvrtech.com
 */

#include "AdvancedConfig.h"
#include <esp_now.h>
#include <WiFi.h>

//This is the configuration file, main structure in _main.ino
//CONFIGURATION SETTINGS:
#define COMMUNICATION COMM_SERIAL //Which communication protocol to use
//serial over USB
  #define SERIAL_BAUD_RATE 115200
  
//serial over Bluetooth
  #define BTSERIAL_DEVICE_NAME "lucidgloves-left"

//ANALOG INPUT CONFIG
#define FLIP_POTS  true  //Flip values from potentiometers (for fingers!) if they are backwards

//Gesture enables, make false to use button override
#define TRIGGER_GESTURE true
#define GRAB_GESTURE    true
#define PINCH_GESTURE   true


//BUTTON INVERT
//If a button registers as pressed when not and vice versa (eg. using normally-closed switches),
//you can invert their behaviour here by setting their line to true.
//If unsure, set to false
#define INVERT_A false
#define INVERT_B false
#define INVERT_JOY false
#define INVERT_MENU false
#define INVERT_CALIB false
//These only apply with gesture button override:
#define INVERT_TRIGGER false
#define INVERT_GRAB false
#define INVERT_PINCH false


//joystick configuration
#define JOYSTICK_BLANK true //make true if not using the joystick
#define JOY_FLIP_X false
#define JOY_FLIP_Y false
#define JOYSTICK_DEADZONE 10 //deadzone in the joystick to prevent drift (in percent)

#define NO_THUMB false //If for some reason you don't want to track the thumb

#define USING_CALIB_PIN true //When PIN_CALIB is shorted (or it's button pushed) it will reset calibration if this is on.

#define USING_FORCE_FEEDBACK false //Force feedback haptics allow you to feel the solid objects you hold
#define SERVO_SCALING false //dynamic scaling of servo motors

#if defined(ESP32)
  //(This configuration is for ESP32 DOIT V1 so make sure to change if you're on another board)
  #define PIN_PINKY     36
  #define PIN_RING      39
  #define PIN_MIDDLE    34
  #define PIN_INDEX     35
  #define PIN_THUMB     32
  #define PIN_WRIST     33
  #define PIN_FOREARM   25
  #define PIN_ELBOW
  #define PIN_JOY_X     20
  #define PIN_JOY_Y     25
  #define PIN_JOY_BTN   26
  #define PIN_A_BTN     20 
  #define PIN_B_BTN     14
  #define PIN_TRIG_BTN  12 //unused if gesture set
  #define PIN_GRAB_BTN  13 //unused if gesture set
  #define PIN_PNCH_BTN  23 //unused if gesture set
  #define PIN_CALIB      0 //button for recalibration (You can set this to GPIO0 to use the BOOT button, but only when using Bluetooth.)
  #define DEBUG_LED 2
  #define PIN_PINKY_MOTOR    5  //used for force feedback
  #define PIN_RING_MOTOR      18 //^
  #define PIN_MIDDLE_MOTOR    19 //^
  #define PIN_INDEX_MOTOR     21 //^
  #define PIN_THUMB_MOTOR     17 //^
  #define PIN_MENU_BTN        27
  
//PINS CONFIGURATION 
#elif defined(__AVR__)
  //(This configuration is for Arduino Nano so make sure to change if you're on another board)
  #define PIN_PINKY     A0
  #define PIN_RING      A1
  #define PIN_MIDDLE    A2
  #define PIN_INDEX     A3
  #define PIN_THUMB     A4
  #define PIN_JOY_X     A6
  #define PIN_JOY_Y     A7
  #define PIN_JOY_BTN   7 
  #define PIN_A_BTN     8 
  #define PIN_B_BTN     9
  #define PIN_TRIG_BTN  10 //unused if gesture set
  #define PIN_GRAB_BTN  11 //unused if gesture set
  #define PIN_PNCH_BTN  12 //unused if gesture set
  #define PIN_CALIB     13 //button for recalibration
  #define DEBUG_LED     LED_BUILTIN
  #define PIN_PINKY_MOTOR     2 //used for force feedback
  #define PIN_RING_MOTOR      3 //^
  #define PIN_MIDDLE_MOTOR    4 //^
  #define PIN_INDEX_MOTOR     5 //^
  #define PIN_THUMB_MOTOR     6 //^
  #define PIN_MENU_BTN        8
#endif


//Test ------------------------------------------------

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);


/*
#define ALWAYS_CALIBRATING CALIBRATION_LOOPS == -1

#define CALIB_OVERRIDE true
#if USING_CALIB_PIN && COMMUNICATION == COMM_SERIAL && PIN_CALIB == 0 && !CALIB_OVERRIDE
  #error "You can't set your calibration pin to 0 over usb. You can calibrate with the BOOT button when using bluetooth only. Set CalibOverride to true to override this."
#endif

//ESPNOW Necessary Items
  #include <esp_now.h>
  #include <WiFi.h>

  //Sheild address
  uint8_t broadcastAddress[] = {0x24, 0xdc, 0xc3, 0x43, 0xc4, 0x98};

  // Structure example to send data
  // Must match the receiver structure
  typedef struct struct_message {
    char a[32];
    char* GloveData;
  } struct_message;

  // Create a struct_message called myData
  struct_message myData;

  esp_now_peer_info_t peerInfo;


  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }


// --- ESPNOW Setup end ---

//ICommunication* comm;
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
    strcpy(myData.a, "LucidGlove data");
    myData.GloveData = encode(fingerPos, getJoyX(), getJoyY(), joyButton, triggerButton, aButton, bButton, grabButton, pinchButton, calibButton, menuButton);
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
    delay(LOOP_TIME);
  }
}
*/