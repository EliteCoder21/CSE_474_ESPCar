#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <semphr.h>

// Define Pin constants
#define TRIG_PIN 5
#define ECHO_PIN 18
#define ALARM_PIN 4

// Set up serial connection to Arduino Mega (motor slave)
HardwareSerial SerialMega(2); // UART2, GPIO16=RX, GPIO17=TX (adjust for your board)

// Setup Semaphores for safety tracking
SemaphoreHandle_t xSafetySemaphore = NULL;

// Setup Task handles for tasks
TaskHandle_t alarmTaskHandle = NULL;
TaskHandle_t driveTaskHandle = NULL;

// Task to run the alarm
void alarmTask(void *parameter) {
  while (1) {
    // Wait indefinitely for someone to give the semaphore
    if (xSemaphoreTake(xSafetySemaphore, portMAX_DELAY) == pdTRUE) {

      // Turn buzzer on
      digitalWrite(ALARM_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(ALARM_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// Task to drive around
void driveTask(void *parameter) {
  while (1) {
    
    // Drive around
    SerialMega.println("FORWARD");
    vTaskDelay(pdMS_TO_TICKS(500));
    SerialMega.println("STOP");
    vTaskDelay(pdMS_TO_TICKS(50));

/*
    SerialMega.println("LEFT");
    delay(1000);

    SerialMega.println("RIGHT");
    delay(1000);

    SerialMega.println("BACKWARD");
    delay(2000);

    SerialMega.println("STOP");
    delay(3000);
*/
  }
}

// Driving Decision Task




// Setup function
void setup() {

  // Start up Serial
  Serial.begin(9600);
  SerialMega.begin(9600, SERIAL_8N1, 16, 2); // RX, TX

  // Setup ultrasonic sensor and beeper pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ALARM_PIN, INPUT);

  // Create Tasks
  xTaskCreatePinnedToCore(
    driveTask,
    "Drive",
    4096,
    NULL,
    1,
    &driveTaskHandle,
    0                       // core 0
  );
  
  xTaskCreatePinnedToCore(
    alarmTask,
    "Alarm",
    4096,
    NULL,
    1,
    &alarmTaskHandle,
    1                       // core 1
  );
  
  Serial.println("All tasks created...");
}

void loop() {
  
}
