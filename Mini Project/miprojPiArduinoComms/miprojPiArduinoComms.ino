#include <Wire.h>
#define MY_ADDR 8
// Global variables to be used for I2C communication
volatile uint8_t sendToDuino = 0;
volatile uint8_t outputString[2] = {0};
volatile uint8_t reply = 0;

void setup() {
  Serial.begin(115200);
  // Control built in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts 
  Wire.onReceive (receive);
  // Send data back
  Wire.onRequest(request);
}

void loop() {
  //If there is data on the buffer, read it
  if (outputString > 0){
    if (offset == 1){
      digitalWrite (LED_BUILTIN, sendToDuino[0]);
    }
    printReceived();
    msgLength = 0;
  }
}

void printReceived(){
  Serial.print("Message received: ");
  Serial.println(sendToDuino);
}

void request(){
  Wire.write(reply);
  reply = 0;
}


