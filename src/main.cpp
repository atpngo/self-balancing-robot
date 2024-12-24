#include <Arduino.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "light.h"
#include "motor_pin_manager.h"
#include "robot.h"
#include "pid_controller.h"
// Settings
static const uint8_t buf_len = 20;


// Globals
static int led_delay = 500;   // ms

Light light(2);

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

// void isrA();
// void isrB();

Robot robot(motorPinsA, motorPinsB);
PID_Controller wheelTrackingController(10, 0, 0);

// // Interrupt Service Routines
void isrA() {
  robot.getMotorA()->updateEncoder();
}

void isrB() {
  robot.getMotorB()->updateEncoder();
}

//*****************************************************************************
// Tasks

// Task: Blink LED at rate set by global variable
void toggleLED(void *parameter) {
  while (1) {
    light.turnOn();
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    light.turnOff();
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

// Task: Read from serial terminal
// Feel free to use Serial.readString() or Serial.parseInt(). I'm going to show
// it with atoi() in case you're doing this in a non-Arduino environment. You'd
// also need to replace Serial with your own UART code for non-Arduino.
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
  while (1) {
    encoderValueA = robot.getMotorA()->getEncoderValue();
    encoderValueB = robot.getMotorB()->getEncoderValue();
    Serial.print("*");
    Serial.print(encoderValueA);
    Serial.print(",");
    Serial.print(encoderValueB);
    Serial.print("\n");
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void spinMotorA(void *parameters) {
  int encoderA;
  int encoderB;
  int err;
  int p = 10;
  int signal;
  while (1) {
    encoderA = robot.getMotorA()->getEncoderValue();
    encoderB = robot.getMotorB()->getEncoderValue();
    signal = wheelTrackingController.getCommand(encoderA, encoderB);
    Serial.print("got this value from PID: ");
    Serial.println(signal);
    if (abs(signal) > 1) {
      robot.getMotorA()->spin(signal);
    }
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}
//*****************************************************************************
// Main

void setup() {
  // Configure pins
  light.configurePins();

  attachInterrupt(digitalPinToInterrupt(motorPinsA.ENCODER_INTERRUPT), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(motorPinsB.ENCODER_INTERRUPT), isrB, RISING);

  // Configure serial and wait a second
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Multi-task LED Demo");
  Serial.println("Enter a number in milliseconds to change the LED delay.");

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
            1024,           // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,           // Parameter to pass
            1,              // Task priority (must be same to prevent lockup)
            NULL,           // Task handle
            1);       // Run on one core for demo purposes (ESP32 only)

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