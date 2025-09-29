# ESP32S3 - DHT22 FreeRTOS
## Notes regarding the assignment 
The Pulseview needed 12MHz and 50M samples to work correctly and demonstrate the timings as desired. 

## Connection
I used the Breadboard and i inserted the DHT22 sensor module to the following:
- Vin -> 3.3V
- Ground -> GND
- Data -> GPIO4

The Logic analyzer connections:
- Ground -> GND
- Channel 1 (D1) -> GPIO2 (heartbeat - 200ms)
- Channel 2 (D2) -> GPIO15 (fastbeat - 1ms)

## Tasks
- DHT22 Task: reads sensor every 5 seconds using ISR
- Serial Task: handles all serial output via message queue  
- Heartbeat Task: toggles GPIO2 every 200ms
- Fastbeat Task: toggles GPIO15 every 1ms (high priority)
- Button Task: monitors boot button for user ID output

All tasks run on Core 1 as required.