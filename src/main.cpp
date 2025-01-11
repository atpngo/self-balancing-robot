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
  // Balancing PID
  float imuP;
  float imuI;
  float imuD;
  // Motor PID
  float motorP;
  float motorI;
  float motorD;
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
  ACTIVATE_SERVO,
  TURN_ON_LED,
  TURN_OFF_LED
};

typedef struct message_to_robot {
  Command command;
  bool updateController;  // set to true when you want to update PID values
  int controllerID; // 0 for balancer, 1 for motor
  // Balancing PID
  float imuP;
  float imuI;
  float imuD;
  // Motor PID
  float motorP; // temporarily use this as a potentiomeetr (lol)
  float motorI;
  float motorD;
} message_to_robot;

// Globals
message_to_robot myData;
message_from_robot outgoingData;

Light led(2);

MotorPinManager motorPinsA {
  .ENCODER_INTERRUPT = 18,
  .ENCODER_SIGNAL    = 19, 
  .MOTOR_IN_A        = 4,  // swap these if the direction is bad
  .MOTOR_IN_B        = 0, 
  .ENABLE            = 16
};

MotorPinManager motorPinsB {
  .ENCODER_INTERRUPT = 33,
  .ENCODER_SIGNAL    = 32,
  .MOTOR_IN_A        = 26, // swap these if direction incorrect
  .MOTOR_IN_B        = 25,
  .ENABLE            = 27
};

const int servoPin = 15;

Robot robot(motorPinsA, motorPinsB);
PID_Controller wheelTrackingController(10, 0, 0);
PID_Controller balancingController(20, 0, 0.5);

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
  switch (myData.command) {
    case (NOP):
      break;
    case (CALIBRATE_IMU):
      break;
    case (ARM):
      robot.arm();
      led.turnOn();
      break;
    case (ABORT):
      robot.disarm();
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
      break;
    case (TURN_ON_LED):
      led.turnOn();
      break;
    case (TURN_OFF_LED):
      led.turnOff();
      break;
    default:
      break;
  }

  // handle received PID values
  if (myData.updateController) {
    if (myData.controllerID == 0) {
      balancingController.setKp(myData.imuP);
      balancingController.setKi(myData.imuI);
      balancingController.setKd(myData.imuD);
    }
    else if (myData.controllerID == 1) {
      // temporary
      robot.setTargetPitch(myData.motorP);
    }
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 

//*****************************************************************************
// Tasks
void printStatusToSerial(void *parameters) {
  while (1) {
    robot.calculateAngles();
    outgoingData.encoderA = robot.getMotorA()->getEncoderValue();
    outgoingData.encoderB = robot.getMotorB()->getEncoderValue();
    outgoingData.pitch = robot.getIMU()->getPitch();
    outgoingData.yaw = robot.getIMU()->getYaw();
    outgoingData.isArmed = robot.isArmed();
    outgoingData.isServoActivated = robot.isServoActivated();
    outgoingData.imuP = balancingController.getKp();
    outgoingData.imuI = balancingController.getKi();
    outgoingData.imuD = balancingController.getKd();
    // outgoingData.motorP = robot.getKp();
    // temporary
    outgoingData.motorP = robot.getTargetPitch();
    outgoingData.motorI = robot.getKi();
    outgoingData.motorD = robot.getKd();
    // Send via ESPNOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));

    if (result == ESP_OK) {
      // Serial.println("Sending confirmed");
    }
    else {
      Serial.println("Sending error");
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

void spinMotorA(void *parameters) {
  int encoderA;
  int encoderB;
  int signal;
  while (1) {
    // TODO: move conditional logic into robot class, fix how we control motors from main
    if (robot.isArmed()) {
      encoderA = robot.getMotorA()->getEncoderValue();
      encoderB = robot.getMotorB()->getEncoderValue();
      signal = wheelTrackingController.getCommand(encoderA, encoderB);
      if (abs(signal) > 1) {
        robot.spinA(-1*signal);
      }
    }
    
    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

void balance(void *parameters) {
  double pitch;
  double command;
  while (1) {
    if (robot.isArmed()) {
      robot.calculateAngles();
      pitch = robot.getIMU()->getPitch();
      command = balancingController.getCommand(pitch, robot.getTargetPitch());
      robot.spinA(command);
      robot.spinB(command);
        // robot.setTargetPosition(command);
        // robot.moveToTargetPosition();
        // robot.getMotorA()->spinToPosition(50);
        // robot.getMotorB()->spinToPosition(200);
      

    }
    
    // look into task delay
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  // while (1) {
  //   robot.spinA(1);
  //   robot.spinB(1);
  //   vTaskDelay(1 / portTICK_PERIOD_MS);

  // }
  
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

  xTaskCreatePinnedToCore(
    balance,
    "Balance the robot",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  
  // xTaskCreatePinnedToCore(
  //   sweepServo,
  //   "Sweep Servo",
  //   1024,
  //   NULL,
  //   1,
  //   NULL,
  //   1
  // );


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