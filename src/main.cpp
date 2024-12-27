#include "Arduino.h"
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "light.h"
#include "motor_pin_manager.h"
#include "robot.h"
#include "pid_controller.h"
#include "WiFi.h"
#include <esp_now.h>


// Test bed MAC:    EC:64:C9:85:4B:F8
// Transmitter MAC: EC:64:C9:85:91:B0
// Settings
static const uint8_t buf_len = 20;
uint8_t broadcastAddress[] = {0xEC, 0x64, 0xC9, 0x85, 0x91, 0xB0}; 
esp_now_peer_info_t peerInfo;

typedef struct message_from_robot {
  float pitch;
  float yaw;
  int encoderA;
  int encoderB;
  bool isArmed;
  bool isServoActivated;
} message_from_robot;

enum Command {
  NOP, 
  CALIBRATE_IMU,
  ARM,
  ABORT,
  FORWARD,
  BACKWARD,
  TURN_CCW,
  TURN_CW,
  ACTIVATE_SERVO
};

typedef struct message_to_robot {
  Command command;
} message_to_robot;

// Globals
static int led_delay = 500;   // ms
message_to_robot myData;
message_from_robot outgoingData;

Light led(2);

MotorPinManager motorPinsA {
  .ENCODER_INTERRUPT = 18,
  .ENCODER_SIGNAL    = 19, 
  .MOTOR_IN_A        = 0,  // swap these if the direction is bad
  .MOTOR_IN_B        = 4, 
  .ENABLE            = 16
};

MotorPinManager motorPinsB {
  .ENCODER_INTERRUPT = 33,
  .ENCODER_SIGNAL    = 32,
  .MOTOR_IN_A        = 25, // swap these if direction incorrect
  .MOTOR_IN_B        = 26,
  .ENABLE            = 27
};


Robot robot(motorPinsA, motorPinsB);
PID_Controller wheelTrackingController(10, 0, 0);

// // Interrupt Service Routines
void isrA() {
  robot.getMotorA()->updateEncoder();
}

void isrB() {
  robot.getMotorB()->updateEncoder();
}

// ESP-NOW Callbacks
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)  {
  memcpy(&myData, incomingData, sizeof(myData));
  // Handle command here
  Serial.println("RECEIVED DATA, WOOT WOOT");
  switch (myData.command) {
    case (NOP):
      break;
    case (CALIBRATE_IMU):
      break;
    case (ARM):
      break;
    case (ABORT):
      break;
    case (FORWARD):
      break;
    case (BACKWARD):
      break;
    case (TURN_CCW):
      break;
    case (TURN_CW):
      break;
    case (ACTIVATE_SERVO):
      led.toggle();
      break;
    default:
      break;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 

//*****************************************************************************
// Tasks

// Task: Blink LED at rate set by global variable
void toggleLED(void *parameter) {
  while (1) {
    led.turnOn();
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    led.turnOff();
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

void readSerial(void *parameters) {

  char c;
  char buf[buf_len];
  uint8_t idx = 0;

  // Clear whole buffer
  memset(buf, 0, buf_len);

  // Loop forever
  while (1) {

    // Read characters from serial
    if (Serial.available() > 0) {
      Serial.print("reading from serial...\n");
      c = Serial.read();
      // Update delay variable and reset buffer if we get a newline character
      if (c == 'x') {
        led_delay = atoi(buf);
        Serial.print("Updated LED delay to: ");
        Serial.println(led_delay);
        memset(buf, 0, buf_len);
        idx = 0;
      } else {
        
        // Only append if index is not over message limit
        if (idx < buf_len - 1) {
          buf[idx] = c;
          idx++;
        }
      }
    }
  }
}

void printStatusToSerial(void *parameters) {
  int encoderValueA;
  int encoderValueB;
  float pitch;
  float yaw;
  bool isArmed;
  bool isServosActivated;
  while (1) {
    encoderValueA = robot.getMotorA()->getEncoderValue();
    encoderValueB = robot.getMotorB()->getEncoderValue();
    robot.calculateAngles();
    pitch = robot.getIMU()->getPitch();
    yaw = robot.getIMU()->getYaw();
    isArmed = robot.getIsArmed();
    isServosActivated = robot.getIsServoActivated();

    outgoingData.encoderA = encoderValueA;
    outgoingData.encoderB = encoderValueB;
    outgoingData.pitch = pitch;
    outgoingData.yaw = yaw;
    outgoingData.isArmed = isArmed;
    outgoingData.isServoActivated = isServosActivated;

    // Send via ESPNOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));

    if (result == ESP_OK) {
      // Serial.println("Sending confirmed");
    }
    else {
      Serial.println("Sending error");
    }
    // Serial.print("*");
    // Serial.print(encoderValueA);
    // Serial.print(",");
    // Serial.print(encoderValueB);
    // Serial.print(",");
    // Serial.print(pitch);
    // Serial.print("\n");
    // if (pitch > 5) {
    //   robot.getMotorB()->spin(Motor::FORWARD, 200); 
    // } else if (pitch < -5) {
    //   robot.getMotorB()->spin(Motor::BACKWARD, 200); 
    // } else {
    //   robot.getMotorB()->stop();
    // }
    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}

void spinMotorA(void *parameters) {
  int encoderA;
  int encoderB;
  int signal;
  while (1) {
    encoderA = robot.getMotorA()->getEncoderValue();
    encoderB = robot.getMotorB()->getEncoderValue();
    signal = wheelTrackingController.getCommand(encoderA, encoderB);
    if (abs(signal) > 1) {
      robot.getMotorA()->spin(signal);
    }
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}


//*****************************************************************************
// Main

void setup() {
  led.turnOff();
  
  

  attachInterrupt(digitalPinToInterrupt(motorPinsA.ENCODER_INTERRUPT), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(motorPinsB.ENCODER_INTERRUPT), isrB, RISING);

  // Configure serial and wait a second
  Wire.begin();
  Wire.setClock(400000); 
  Serial.begin(115200);
  while (!Serial); 

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register the send callback
  esp_now_register_recv_cb(OnDataRecv);
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

  robot.initialize();
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  led.turnOn();
  // // Start blink task
  // xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
  //           toggleLED,      // Function to be called
  //           "Toggle LED",   // Name of task
  //           1024,           // Stack size (bytes in ESP32, words in FreeRTOS)
  //           NULL,           // Parameter to pass
  //           1,              // Task priority
  //           NULL,           // Task handle
  //           0);       // Run on one core for demo purposes (ESP32 only)
            
  // // Start serial read task
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    printStatusToSerial,     // Function to be called
    "Print to Serial",  // Name of task
    4096,           // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,           // Parameter to pass
    1,              // Task priority (must be same to prevent lockup)
    NULL,           // Task handle
    1
  );   



  // Motor tracking
  xTaskCreatePinnedToCore(
    spinMotorA,
    "Spin motor A",
    1024,
    NULL,
    1,
    NULL,
    0
  );

  // // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}