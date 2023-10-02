//Description: This code is what controls the wheel to turn to the specific angle requested using the 
//             motors current position and some PI controllers to manage the rise time and overshoot.
//Hardware:    You will need an Arduino Uno connected to a motor, a voltage monitor, and Raspberry Pi.
//             The motor should have a wheel attached and the Raspberry Pi will need a camera.


#include <Wire.h> //This is our Pi communication library
#define A2 2
#define B2 5
#define PWMPIN 9  //Motor PWM
#define DIRPIN 7  //Motor Direction
#define PIN4 4
#define MY_ADDR 8
// This is defining all our pins

/*
1   2

4   3

1 -> 0
2 -> pi/2
3 -> pi
4 -> 3pi/2

*/
bool debug = 0;
float Kp = 4; // 1.7 was Our gain we found in our insitiall simulink
float Kp_pos = 5; // The proportional gain
float Ki_pos = 0.75; // the intergral gain

int targetNum = 1;
int lastTargetNum;
int targetCnts;
float targetRad;
float desired_speed;
float integral_error = 0;
float pos_error = 0;
float error = 0;

int value;
int motor_dir = 1;
long encoderCounts = 0;
int lastA2, lastB2;
unsigned long desired_Ts_ms = 10;  // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
float last_time;

float ang_velocity;
float last_pos_rad;

char txBuf[7];
char rxBuf[7];
int rxInd = 0;
int reply = 0;

void A2Change() {                            //Interrupt handler for pin A
  if (digitalRead(A2) == digitalRead(B2)) {  //Encoder direction check
    encoderCounts += 2;                      //Adjusts for changes made during periodic reporting
  } else {
    encoderCounts -= 2;
  }
  lastA2 = digitalRead(A2);  //Reset states
  lastB2 = digitalRead(B2);
}

void setup() {
  Serial.begin(115200);     // Set the baud rate fast so that we can display the results
  last_time_ms = millis();  // set up sample time variable
  start_time_ms = last_time_ms;
  pinMode(A1, INPUT_PULLUP);  //Set inputs and pullups
  pinMode(B1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);  //Set inputs and pullups
  pinMode(B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(A2), A2Change, CHANGE);  // attach interrupt handler

  pinMode(PWMPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);

  pinMode(PIN4, OUTPUT);
  digitalWrite(PIN4, HIGH);

  Serial.begin(115200);
  last_time_ms = millis();  // set up sample time variable
  start_time_ms = last_time_ms;
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(I2Creceive);
  // Send data back
  Wire.onRequest(I2Crequest);
}

void loop() {



  if (rxInd > 0) { //Check for incoming data
    // read the incoming byte:
    char incomingByte = rxBuf[0]; //Assumes only one character will be transmitted
    rxInd = 0; //Resets indication that buffer has data
    Serial.print("Read ");
    Serial.print(incomingByte);
    Serial.println(" from Pi");
    updateTarget(incomingByte); //Update target location for PI controller
    if (incomingByte == 'd') { //Toggle debug prints
      debug = !debug;
    } else if (incomingByte == 'S') { //Reset timer and full throttle motor - for parameter tuning
      current_time = 0.0;
      last_time_ms = millis();  // reset sample time variable
      start_time_ms = last_time_ms;
      value = 100.0;
    } 
    else if (incomingByte == '1' or incomingByte == '2' or incomingByte == '3' or incomingByte == '4') { }
  }

  float pos_rad;

  current_time = (float)(last_time_ms - start_time_ms) / 1000; //Current time (sec)

  pos_rad = 2 * PI * (float)encoderCounts / 3200.0;  //Convert positions from steps to radians

  ang_velocity = (pos_rad - last_pos_rad) / (current_time - last_time); //Calculate angular velocity


  pos_error = targetRad - pos_rad;  //Calc position error
  integral_error = integral_error + pos_error * ((float)desired_Ts_ms / 1000); //Discrete integration (just a sum)
  desired_speed = Kp_pos * pos_error + Ki_pos * integral_error; //Calc desired speed
  error = desired_speed - ang_velocity; //Velocity error
  value = Kp * error; //Final PWM value calculation

  if (value < 0) { //Translate negative pwm value to reversed motor direction
    value *= -1;
    motor_dir = 0;
  } else {
    motor_dir = 1;
  }
  
  value = value * 255 / 8; //Scale control loop output to PWM
  if (value > 255) { value = 255; } //Cap PI controller output at 255


  analogWrite(PWMPIN, value); //Control direction pin
  if (motor_dir) {
    digitalWrite(DIRPIN, HIGH);
  } else {
    digitalWrite(DIRPIN, LOW);
  }

  dtostrf(last_pos_rad, 3, 2, txBuf); //Float to string for reporting back to pi -Defunct
  //Wire.write(txBuf);
  Serial.println(txBuf); //Alt Debug

  if (debug) {
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(value);
    Serial.print("\t");
    Serial.print(ang_velocity);
    Serial.println("");
  }

  last_pos_rad = pos_rad;

  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
  last_time = current_time;
}

//Translate the previous target and current target to a step target adjustment 
void updateTarget(char tgtCmd) {
  lastTargetNum = targetNum; //
  targetNum = tgtCmd - '0'; //Char to int
  if (targetNum == 1 && lastTargetNum == 4) { // 4 to 1 transition
    targetCnts -= 800;
  } else if (targetNum == 4 && lastTargetNum == 1) { // 1 to 4 transition
    targetCnts += 800;
  } else if (targetNum > lastTargetNum + 1) { // 1 to 3 transition and 2 to 4 transition
    targetCnts -= 1600;
  } else if (targetNum < lastTargetNum - 1) { //3 to 1 transition and 4 to 2 transition
    targetCnts += 1600;
  } else if (targetNum > lastTargetNum) { //1 to 2, 2 to 3, 3 to 4 transitions
    targetCnts -= 800;
  } else if (targetNum < lastTargetNum) { //4 to 3, 3 to 2, 2 to 1 transitions
    targetCnts += 800;
  }
  targetRad = 2 * PI * (float)targetCnts / 3200.0; //Convert to radians for PI loop
}

//Handle I2C reception from pi
void I2Creceive() {
  Wire.read();
  while (Wire.available()) {
    rxBuf[rxInd] = Wire.read();
    rxInd++;
  }
}

// Acknowledge pi communication requests
void I2Crequest() {
  Wire.write(reply);
  reply = 0;
}
