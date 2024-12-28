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
// #include "ESP32Servo.h"


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
  float p;
  float i;
  float d;
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
  float p;
  float i;
  float d;
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

const int servoPin = 15;

Robot robot(motorPinsA, motorPinsB);
PID_Controller wheelTrackingController(10, 0, 0);
PID_Controller balancingController(50, 0, 0);
// Servo servo;

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
      robot.abort();
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
    balancingController.setKp(myData.p);
    balancingController.setKi(myData.i);
    balancingController.setKd(myData.d);
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


void printStatusToSerial(void *parameters) {
  while (1) {
    robot.calculateAngles();
    outgoingData.encoderA = robot.getMotorA()->getEncoderValue();
    outgoingData.encoderB = robot.getMotorB()->getEncoderValue();
    outgoingData.pitch = robot.getIMU()->getPitch();
    outgoingData.yaw = robot.getIMU()->getYaw();
    outgoingData.isArmed = robot.isArmed();
    outgoingData.isServoActivated = robot.isServoActivated();
    outgoingData.p = balancingController.getKp();
    outgoingData.i = balancingController.getKi();
    outgoingData.d = balancingController.getKd();
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
        robot.spinA(signal);
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
      if (abs(pitch) > 30.0) {
        robot.abort();
      } else {  
        command = balancingController.getCommand(pitch, 0.0);
        robot.spinA(command);
        robot.spinB(command);
      }

    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

}

// void sweepServo(void *parameters) {
//   int pos = 0;
//   while (1) {
//     servo.write(0);
//     vTaskDelay(1000/ portTICK_PERIOD_MS);
//     servo.write(90);
//     vTaskDelay(1000/ portTICK_PERIOD_MS);
//     servo.write(180);
//     vTaskDelay(1000/ portTICK_PERIOD_MS);

//     // for (pos = 0; pos<180; pos++) {
//     //   servo.write(pos);
//     //   vTaskDelay(15 / portTICK_PERIOD_MS);
//     // }
//     // // sweep back
//     // for (pos = 180; pos >= 0; pos--) {
//     //   servo.write(pos);
//     //   vTaskDelay(15 / portTICK_PERIOD_MS);
//     // }
//   }
// }


//*****************************************************************************
// Main

void setup() {
  led.turnOff();
  
  // ESP32PWM::allocateTimer(0);
	// ESP32PWM::allocateTimer(1);
	// ESP32PWM::allocateTimer(2);
	// ESP32PWM::allocateTimer(3);
  // servo.setPeriodHertz(50);
  // servo.attach(servoPin, 500, 2400);

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
  // xTaskCreatePinnedToCore(
  //   spinMotorA,
  //   "Spin motor A",
  //   1024,
  //   NULL,
  //   1,
  //   NULL,
  //   0    
  // );

  // // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}