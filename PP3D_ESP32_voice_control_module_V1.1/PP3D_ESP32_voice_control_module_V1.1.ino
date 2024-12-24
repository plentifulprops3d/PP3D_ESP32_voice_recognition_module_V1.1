#include "DFRobot_DF2301Q.h"
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h> // Using ESP32Servo.h library for servo control

#define Led 10        // GPIO4 for LED
#define RX_PIN 5      // GPIO5 for RX (D/T pin on module)
#define TX_PIN 6      // GPIO6 for TX (C/R pin on module)
#define BUTTON_PIN_1 1 // Button 1 to trigger servo control
#define BUTTON_PIN_2 2
#define BUTTON_PIN_3 3
#define BUTTON_PIN_4 4

#if defined(ESP32)  // Use the hardware serial with remappable pin: Serial1
DFRobot_DF2301Q_UART asr(&Serial1, RX_PIN, TX_PIN);
#else
SoftwareSerial softSerial(RX_PIN, TX_PIN);
DFRobot_DF2301Q_UART asr(&softSerial);
#endif

uint8_t receiverMac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // Receiver MAC address
uint8_t soundBoardMAC[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Sound effect board MAC
unsigned long lastPressTime = 0;  
unsigned long debounceDelay = 300;  

void setup() {
  Serial.begin(115200);

  Serial.print("Control Board MAC Address: ");
  Serial.println(WiFi.macAddress());

  pinMode(Led, OUTPUT);
  digitalWrite(Led, LOW);

  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(BUTTON_PIN_3, INPUT_PULLUP);
  pinMode(BUTTON_PIN_4, INPUT_PULLUP);

  while (!(asr.begin())) {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Voice module initialized");

  asr.settingCMD(DF2301Q_UART_MSG_CMD_SET_MUTE, 0);
  asr.settingCMD(DF2301Q_UART_MSG_CMD_SET_VOLUME, 7);
  asr.settingCMD(DF2301Q_UART_MSG_CMD_SET_WAKE_TIME, 20);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized");

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  memcpy(peerInfo.peer_addr, soundBoardMAC, 6);
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // Button 1 press (GPIO 1) will trigger servo control commands (101 and 102)
  if (digitalRead(BUTTON_PIN_1) == LOW) {
    unsigned long currentTime = millis();
    if (currentTime - lastPressTime > debounceDelay) {
      lastPressTime = currentTime;
      // Sending servo control commands (101 to open, 102 to close)
      sendEspNowData(receiverMac, 101);  // Open servo
      Serial.println("Button pressed: Toggling servos to open");
    }
  }

  // Reading voice recognition module commands (asr)
  uint8_t CMDID = asr.getCMDID();
  Serial.print("Received CMDID: ");
  Serial.println(CMDID);

  switch (CMDID) {
    case 5:  // Command for opening servos
      sendEspNowData(receiverMac, 101);
      Serial.println("Voice Command: Opening servos");
      break;
    case 6:  // Command for closing servos
      sendEspNowData(receiverMac, 101);
      Serial.println("Voice Command: Closing servos");
      break;
    case 7:  // Command for triggering sound effect 4
      sendEspNowData(receiverMac, 4);
      Serial.println("Voice Command: Triggering sound effect 4");
      break;
    case 8:  // Command for triggering sound effect 4
      sendEspNowData(receiverMac, 5);
      Serial.println("Voice Command: Triggering sound effect 5");
      break;
    case 9:  // Command for triggering sound effect 4
      sendEspNowData(receiverMac, 99);
      Serial.println("Voice Command: Stop tracks");
      break;
    case 1:  // Wake word detected, trigger sound effect 1
      sendEspNowData(soundBoardMAC, 1); 
      Serial.println("Voice Command: Triggering sound effect 1");
      break;
    default:
      break;
  }

  delay(300);  // Short delay for stable input handling
}

void sendEspNowData(uint8_t* macAddress, uint8_t data) {
  if (esp_now_send(macAddress, &data, sizeof(data)) == ESP_OK) {
    Serial.print("Sent data: ");
    Serial.println(data);
  } else {
    Serial.println("Failed to send data");
  }
}

