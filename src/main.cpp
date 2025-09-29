#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// pins
#define DHT_PIN 4
#define HEARTBEAT_LED_PIN 2
#define FASTBEAT_LED_PIN 15
#define BOOT_BUTTON_PIN 0
#define NEOPIXEL_PIN 38

#define USER_ID "io25m010"

// data structures
struct DHT22Data {
  float temperature;
  float humidity;
  uint32_t readingNumber;
  bool isValid;
};

enum MessageType { 
  MSG_DHT_DATA, 
  MSG_USER_ID, 
  MSG_ERROR 
};

struct SerialMessage {
  MessageType type;
  DHT22Data dhtData;
  char message[64];
};

// globals
// FreeRTOS handles
QueueHandle_t dhtToSerialQueue;
QueueHandle_t dhtRawDataQueue;
QueueHandle_t buttonEventQueue;

// DHT22 ISR variables
volatile bool dataReady = false;
volatile uint32_t lastTime = 0;
volatile uint8_t bitCount = 0;
volatile uint8_t dhtData[5] = {0}; // 40 bits = 5 bytes
volatile bool readingData = false;
volatile bool timeoutOccurred = false;

// Hardware timer for timeout management
hw_timer_t *timeoutTimer = nullptr;

// Task handles for core pinning
TaskHandle_t dhtTaskHandle;
TaskHandle_t serialTaskHandle;
TaskHandle_t heartbeatTaskHandle;
TaskHandle_t fastbeatTaskHandle;
TaskHandle_t buttonTaskHandle;

// utility functions
void setNeoPixel(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(NEOPIXEL_PIN, r, g, b);
}

// dht22 isr functions
// Hardware timer ISR - called when timeout occurs
void IRAM_ATTR timeoutISR() {
  timeoutOccurred = true;
}

// Interrupt handler - called on every pin state change
void IRAM_ATTR dhtInterrupt() {
  uint32_t currentTime = micros();
  
  if (!readingData) return;
  
  uint32_t duration = currentTime - lastTime;
  lastTime = currentTime;
  
  bitCount++;
  
  // Check if this is a HIGH pulse (the one we measure for bit value)
  if (digitalRead(DHT_PIN) == LOW) {
    // Falling edge - end of HIGH pulse, now we can determine the bit value
    if (bitCount >= 3 && bitCount <= 82) {
      uint8_t databitIndex = (bitCount - 3) / 2;
      if (databitIndex < 40) {
        uint8_t byteIndex = databitIndex / 8;
        uint8_t bitPosition = 7 - (databitIndex % 8); // MSB first
        
        // Determine if it's a '1' or '0' based on HIGH pulse duration
        if (duration > 50) { // Threshold: >50μs = bit '1', <50μs = bit '0'
          dhtData[byteIndex] |= (1 << bitPosition);
        }
      }
    }
    
    // Check if we've received all 40 data bits
    if (bitCount >= 82) {
      dataReady = true;
      readingData = false;
    }
  }
}

// dht22 read
void dht22Task(void *parameter) {
  SerialMessage msg;
  static uint32_t readingCount = 0;
  
  // Initialize timeout timer
  timeoutTimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1MHz), count up
  timerAttachInterrupt(timeoutTimer, &timeoutISR, false);
  
  // Initialize - wait for first reading
  vTaskDelay(pdMS_TO_TICKS(2000)); // Give DHT22 time to stabilize
  
  for (;;) {
    setNeoPixel(40, 0, 0); // Red - reading in progress
    
    // Reset variables
    dataReady = false;
    readingData = false;
    bitCount = 0;
    timeoutOccurred = false;
    memset((void*)dhtData, 0, 5); // Clear data array
    
    // Configure and start timeout timer (100ms)
    timerWrite(timeoutTimer, 0); // Reset timer counter
    timerAlarmWrite(timeoutTimer, 100000, false); // 100ms timeout
    timerAlarmEnable(timeoutTimer);
    
    // Send start signal to DHT22
    pinMode(DHT_PIN, OUTPUT);
    digitalWrite(DHT_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(20)); // 20ms delay (DHT22 needs at least 18ms)
    digitalWrite(DHT_PIN, HIGH);
    delayMicroseconds(40); // 40μs delay
    
    pinMode(DHT_PIN, INPUT_PULLUP);
    
    readingData = true;
    lastTime = micros();
    attachInterrupt(digitalPinToInterrupt(DHT_PIN), dhtInterrupt, CHANGE);
    
    // Wait for data or timeout
    while (!dataReady && !timeoutOccurred) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Cleanup
    timerAlarmDisable(timeoutTimer);
    detachInterrupt(digitalPinToInterrupt(DHT_PIN));
    
    if (timeoutOccurred) {
      setNeoPixel(40, 40, 0); // Yellow - timeout
      msg.type = MSG_ERROR;
      strcpy(msg.message, "DHT22 timeout error!");
      xQueueSend(dhtToSerialQueue, &msg, 0);
    } else if (dataReady) {
      // Verify checksum
      uint8_t checksum = dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3];
      
      if (checksum == dhtData[4]) {
        setNeoPixel(0, 40, 0); // Green - success
        
        // Parse data
        uint16_t humidity_raw = (dhtData[0] << 8) | dhtData[1];
        float humidity = humidity_raw / 10.0;
        
        uint16_t temperature_raw = (dhtData[2] << 8) | dhtData[3];
        float temperature = temperature_raw / 10.0;
        
        // Handle negative temperature
        if (dhtData[2] & 0x80) {
          temperature *= -1;
        }
        
        // Send to serial task
        msg.type = MSG_DHT_DATA;
        msg.dhtData.temperature = temperature;
        msg.dhtData.humidity = humidity;
        msg.dhtData.readingNumber = readingCount++;
        msg.dhtData.isValid = true;
        
        xQueueSend(dhtToSerialQueue, &msg, 0);
      } else {
        setNeoPixel(40, 0, 40); // Magenta - checksum error
        msg.type = MSG_ERROR;
        strcpy(msg.message, "DHT22 checksum error!");
        xQueueSend(dhtToSerialQueue, &msg, 0);
      }
    }
    
    // Wait 5 seconds before next reading
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// serial communication 
void serialTask(void *parameter) {
  SerialMessage msg;
  
  for (;;) {
    if (xQueueReceive(dhtToSerialQueue, &msg, portMAX_DELAY) == pdTRUE) {
      switch (msg.type) {
        case MSG_DHT_DATA:
          Serial.printf("%lu: %.1f%% %.1fC\n", 
                       msg.dhtData.readingNumber, 
                       msg.dhtData.humidity, 
                       msg.dhtData.temperature);
          break;
          
        case MSG_USER_ID:
          Serial.println(USER_ID);
          break;
          
        case MSG_ERROR:
          Serial.println(msg.message);
          break;
      }
    }
  }
}

// heartbeat
void heartbeatTask(void *parameter) {
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  
  for (;;) {
    digitalWrite(HEARTBEAT_LED_PIN, !digitalRead(HEARTBEAT_LED_PIN));
    vTaskDelay(pdMS_TO_TICKS(200)); // 200ms
  }
}

// fastbeat
void fastbeatTask(void *parameter) {
  pinMode(FASTBEAT_LED_PIN, OUTPUT);
  
  for (;;) {
    digitalWrite(FASTBEAT_LED_PIN, !digitalRead(FASTBEAT_LED_PIN));
    vTaskDelay(pdMS_TO_TICKS(1)); // 1ms
  }
}

// button 
void buttonTask(void *parameter) {
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  bool lastButtonState = HIGH;
  SerialMessage msg;
  
  for (;;) {
    bool currentButtonState = digitalRead(BOOT_BUTTON_PIN);
    
    // Detect button press (falling edge)
    if (lastButtonState == HIGH && currentButtonState == LOW) {
      msg.type = MSG_USER_ID;
      xQueueSend(dhtToSerialQueue, &msg, 0);
    }
    
    lastButtonState = currentButtonState;
    vTaskDelay(pdMS_TO_TICKS(50)); // 50ms debounce delay
  }
}

// setup function
void setup() {
  Serial.begin(115200);
  Serial.println("FreeRTOS DHT22 System Starting...");
  
  // Initialize NeoPixel
  setNeoPixel(0, 0, 40); // Blue - initializing
  
  // Initialize DHT22 pin
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, HIGH); // Start with HIGH (idle state)
  delay(2000); // Give DHT22 time to initialize
  
  // Create message queues
  dhtToSerialQueue = xQueueCreate(10, sizeof(SerialMessage));
  
  if (dhtToSerialQueue == NULL) {
    Serial.println("Failed to create message queue!");
    return;
  }
  
  // Create tasks on Core 1 (as required by assignment)
  xTaskCreatePinnedToCore(dht22Task, "DHT22Task", 8192, NULL, 2, &dhtTaskHandle, 1);
  xTaskCreatePinnedToCore(serialTask, "SerialTask", 4096, NULL, 1, &serialTaskHandle, 1);
  xTaskCreatePinnedToCore(heartbeatTask, "HeartbeatTask", 2048, NULL, 1, &heartbeatTaskHandle, 1);
  xTaskCreatePinnedToCore(fastbeatTask, "FastbeatTask", 2048, NULL, 3, &fastbeatTaskHandle, 1); // High priority
  xTaskCreatePinnedToCore(buttonTask, "ButtonTask", 2048, NULL, 1, &buttonTaskHandle, 1);
  
  Serial.println("All tasks created successfully!");
  Serial.println("System running on Core 1");
  Serial.println("DHT22 readings every 5 seconds...");
}

// loop function
void loop() {
  // FreeRTOS handles it so no need to insert anything ghere
}