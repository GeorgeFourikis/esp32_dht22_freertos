# Assignment FreeRTOS with DHT22. 
```
Integrate the DHT22 Sensor into a FreeRTOS application
- Create a task which reads the DHT22 data
- Option 1
  - Implement the sensor access yourself
  - You can use an ISR to ensure exact timing of the DHT22 signals
  - Use a message queue to forward the input events from the ISR to the DHT22 task
  - Do not execute the processing of the input events directly in the ISR but use the DHT22 task!
- Option 2
  - Use an external library (make sure it works correctly with FreeRTOS)
- Use a second task for controlling the serial port
- Forward the converted DHT22 data from the DHT22 task to the serial task using a message queue
- Send the same data string as in the second homework to the PC via the serial port
- Only the serial task is allowed to access the serial port!
- Implement a heartbeat task
- Toggles a pin every 200 ms
- Implement a fastbeat task (high priority)
- Toggle a pin very 1 ms
- Add another task which handles the boot button
- If the button is pressed, it triggers the serial task to send your user id (io25mxxx) to the PC via the serial port
- All tasks must run on the same core of the ESP32.
```