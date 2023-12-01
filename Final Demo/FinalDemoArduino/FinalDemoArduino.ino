//Description: This code is what controls what state the wheels are in. This is done by communicating
//             with the Raspberry Pi to find a circle a Marker
//Hardware:    You will need an Arduino Uno connected to a motor, a voltage monitor, and Raspberry Pi.
//             The motor should have a wheel attached and the Raspberry Pi will need a camera.


#include <Wire.h>   //This is our Pi communication library
#define A2 2        //Left Motor Encoder
#define B2 5        //Left Motor Encoder
#define A1 3        //Right Motor Encoder
#define B1 6        //Right Motor Encoder
#define PWMPINL 9   //Left Motor PWM
#define PWMPINR 10  //Right Motor PWM
#define DIRPINL 7   //Left Motor Direction
#define DIRPINR 8   //Right Motor Direction
#define PIN4 4
#define MY_ADDR 8

// START BAILEY ADD
volatile uint8_t instruction[32] = { 0 };
volatile uint8_t reply = 0;
volatile uint8_t msgLength = 0;
// END BAILEY ADD

int talkToPi;
bool debug = 0;
float Kp = 60;  // 1.7 was Our gain we found in our insitiall simulink
float Kpv = 2;
float Kp_pos = 2;    // The proportional gain
float Ki_pos = 0;  // the intergral gain

float readAngle;
float readDistance;
float lastReadAngle;
float lastReadDistance;
int State = 0;
int returnState;
int reset = 1;
int Demo = 2;

float wheelCir = 0.471;
float wheelDiameter = 0.345;
float wheelRadius = 0.163;
float targetrotm;
float targetrotm2 [2];
float targetRad[2];
float currentRad[2];

float desiredTime = 5;
float desiredRadius = 0.36576;
float leftWheelRadius = desiredRadius - wheelRadius;
float rightWheelRadius =  desiredRadius + wheelRadius;;
float rightWheelCirc = rightWheelRadius * 2 * PI;
float leftWheelCirc = leftWheelRadius * 2 * PI;

float desired_speed[2];
float integral_error[2] = { 0, 0 };
float pos_error[2] = { 0, 0 };
float error[2] = { 0, 0 };

int value[2];
int motor_dir[2] = { 1, 1 };
long encoderCounts[2];
int lastA2, lastB2, lastA1, lastB1;
unsigned long desired_Ts_ms = 10;  // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
float last_time;

float ang_velocity[2];
float pos_rad[2];
float last_pos_rad[2];

char txBuf[7];
char rxBuf[7];
int rxInd = 0;
//int reply = 0;

void A2Change() {                            //Interrupt handler for pin A
  if (digitalRead(A2) == digitalRead(B2)) {  //Encoder direction check
    encoderCounts[0] -= 2;                   //Adjusts for changes made during periodic reporting
  } else {
    encoderCounts[0] += 2;
  }
  lastA2 = digitalRead(A2);  //Reset states
  lastB2 = digitalRead(B2);
}

void A1Change() {                            //Interrupt handler for pin A
  if (digitalRead(A1) == digitalRead(B1)) {  //Encoder direction check
    encoderCounts[1] += 2;                   //Adjusts for changes made during periodic reporting
  } else {
    encoderCounts[1] -= 2;
  }
  lastA1 = digitalRead(A1);  //Reset states
  lastB1 = digitalRead(B1);
}

void setup() {
  Serial.begin(115200);     // Set the baud rate fast so that we can display the results
  last_time_ms = millis();  // set up sample time variable
  start_time_ms = last_time_ms;
  pinMode(A1, INPUT_PULLUP);  //Set inputs and pullups
  pinMode(B1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);  //Set inputs and pullups
  pinMode(B2, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(A2), A2Change, CHANGE);  // attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(A1), A1Change, CHANGE);  // attach interrupt handler

  pinMode(PWMPINL, OUTPUT);
  pinMode(DIRPINL, OUTPUT);

  pinMode(PWMPINR, OUTPUT);
  pinMode(DIRPINR, OUTPUT);

  pinMode(PIN4, OUTPUT);
  digitalWrite(PIN4, HIGH);

  //START BAILEY ADD
  //We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(I2Creceive);
  // Send data back
  Wire.onRequest(I2Crequest);
  // END BAILEY ADD
  talkToPi = 8;

  Serial.println(3200 * (leftWheelCirc / wheelCir));
}

void loop() {

  // Reading Angle and Distance from Marker


  current_time = (float)(last_time_ms - start_time_ms) / 1000;  //Current time (sec)

  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if (State == 1) {
    talkToPi = 0;
    digitalWrite(DIRPINL, HIGH);   //Control direction
    digitalWrite(DIRPINR, HIGH);  //Control direction

    analogWrite(PWMPINL, 35);  //Control direction speed
    analogWrite(PWMPINR, 35);  //Control direction speed

    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  } else if (State == 6) {

    digitalWrite(DIRPINL, LOW);  //Control direction
    digitalWrite(DIRPINR, HIGH);  //Control direction

    analogWrite(PWMPINL, 100);  //Control direction speed
    analogWrite(PWMPINR, 110);  //Control direction speed
    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  } else if (State == 7) {

    digitalWrite(DIRPINL, LOW);  //Control direction
    digitalWrite(DIRPINR, HIGH);  //Control direction

    analogWrite(PWMPINL, 110);  //Control direction speed
    analogWrite(PWMPINR, 100);  //Control direction speed
    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  } else if (State == 2) {

    digitalWrite(DIRPINL, LOW);  //Control direction
    digitalWrite(DIRPINR, HIGH);  //Control direction

    analogWrite(PWMPINL, 100);  //Control direction speed
    analogWrite(PWMPINR, 100);  //Control direction speed
    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  } else if (State == 3) {
    analogWrite(PWMPINL, 0);  //Control direction speed
    analogWrite(PWMPINR, 0);  //Control direction speed
    if (Demo == 2) {
      talkToPi = 1;
    }
  } else if (State == 4) {

    if (reset == 1) {
      encoderCounts[0] = 0;
      encoderCounts[1] = 0;
      integral_error[0] = 0;
      integral_error[1] = 0;
      pos_rad[0] = 0;
      pos_rad[1] = 0;
      float targetdeg = 90.0;
      targetrotm = (targetdeg / 360.0) * (wheelDiameter * PI);
      targetRad[1] = (2 * PI) * (targetrotm / wheelCir);
      targetRad[0] = -1 * targetRad[1];
      //Serial.println(targetRad);
      reset = 2;
    }

    for (int i = 0; i < 2; i++) {
      pos_rad[i] = 2 * PI * (float)encoderCounts[i] / 3200.0;  //Convert positions from steps to radians

      ang_velocity[i] = (pos_rad[i] - last_pos_rad[i]) / (current_time - last_time);  //Calculate angular velocity

      pos_error[i] = (-1) * targetRad[i] - pos_rad[i];

      integral_error[i] = integral_error[i] + pos_error[i] * ((float)desired_Ts_ms / 1000);  //Discrete integration (just a sum)
      
      //Serial.println(targetRad);

      if (integral_error[i] > 1.5) { integral_error[i] = 1.5; }
      if (integral_error[i] < -1.5) { integral_error[i] = -1.5; }

      desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];  //Calc desired speed
      error[i] = desired_speed[i] - ang_velocity[i];                          //Velocity error
      value[i] = Kp * error[i];

      if (value[i] < 0) {  //Translate negative pwm value to reversed motor direction
        value[i] *= -1;
        motor_dir[i] = 0;
      } else {
        motor_dir[i] = 1;
      }

      //value[i] = value[i] * 32;                //Scale control loop output to PWM
      if (value[i] > 75) { value[i] = 75; }  //Cap PI controller output at 255

      if (motor_dir[0]) {
        digitalWrite(DIRPINL, LOW);
      } else {
        digitalWrite(DIRPINL, HIGH);
      }
      if (motor_dir[1]) {
        digitalWrite(DIRPINR, HIGH);
      } else {
        digitalWrite(DIRPINR, LOW);
      }

      currentRad[i] = 2 * PI * ((float)encoderCounts[i] / 3200);

      //Serial.println(targetRad[i] + currentRad[i]);

      analogWrite(PWMPINL, value[0]);  //Control direction speed
      analogWrite(PWMPINR, value[1]);  //Control direction speed

      if (targetRad[1] + currentRad[1] < 0.4 && targetRad[0] + currentRad[0] > -0.4){
        talkToPi = 3;
        //State = 5;
      }

    }
    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  } else if (State == 5) {
    
    targetRad[1] = (2 * PI) * (rightWheelCirc / wheelCir);
    targetRad[0] = (2 * PI) * (leftWheelCirc / wheelCir);

    //Serial.println(targetRad[0]);

    desired_speed[0] = targetRad[0] / desiredTime;
    desired_speed[1] = targetRad[1] / desiredTime;

    if (reset == 2) {
      encoderCounts[0] = 0;
      encoderCounts[1] = 0;
      pos_rad[0] = 0;
      pos_rad[1] = 0;
      reset = 0;
    }

    for (int i = 0; i < 2; i++) {
      pos_rad[i] = 2 * PI * (float)encoderCounts[i] / 3200.0;                         //Convert positions from steps to radians
      ang_velocity[i] = (pos_rad[i] - last_pos_rad[i]) / (current_time - last_time);  //Calculate angular velocity

      error[i] = desired_speed[i] - ang_velocity[i];  //Velocity error
      value[i] = Kp * error[i];

      //Serial.println((targetRad[0] / (2 * PI)) * 3200);

      if (value[i] < 0) {  //Translate negative pwm value to reversed motor direction
        value[i] *= -1;
        motor_dir[i] = 0;
      } else {
        motor_dir[i] = 1;
      }

      if (value[i] > 255) { value[i] = 255; }  //Cap PI controller output at 255

      analogWrite(PWMPINL, value[0]);  //Control direction speed
      analogWrite(PWMPINR, value[1]);  //Control direction speed

      if (motor_dir[0]) {
        digitalWrite(DIRPINL, LOW);
      } else {
        digitalWrite(DIRPINL, HIGH);
      }
      if (motor_dir[1]) {
        digitalWrite(DIRPINR, HIGH);
      } else {
        digitalWrite(DIRPINR, LOW);
      }
    }
    if (encoderCounts[0] >= (7450)){
      State = 9;
      analogWrite(PWMPINL, 0);  //Control direction speed
      analogWrite(PWMPINR, 0);
    }
  }
  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  if (debug) {
    //  Serial.print(current_time);
    //  Serial.print("\t");
    //  Serial.print(value[0]);
    //  Serial.print("\t");
    //  Serial.print(encoderCounts[0]);
    //  Serial.print("\t");
    //  Serial.print(value[1]);
    //  Serial.print("\t");
    //  Serial.print(encoderCounts[1]);
    //  Serial.println("");
  }

  //Serial.println(State);

  last_pos_rad[0] = pos_rad[0];
  last_pos_rad[1] = pos_rad[1];

  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
  last_time = current_time;

  //BAILEY ADD START
  // If there is data on the buffer, read it
  if (msgLength > 0) {
    digitalWrite(LED_BUILTIN, instruction[0]);
    printReceived();
    msgLength = 0;
  }
  //BAILEY ADD END
}  //End Loop


//BAILEY ADD START
void printReceived() {
  //Serial.print("Instruction received:");
  State = instruction[0];
  //Serial.print(State);
  //Serial.println("");
}

//Handle I2C reception from pi
void I2Creceive() {
  //Serial.println("I");
  // offset = Wire.read();
  while (Wire.available()) {
    //rxBuf[rxInd] = Wire.read();
    //rxInd++;
    instruction[msgLength] = Wire.read();
    Serial.println(String(instruction[msgLength]));
    msgLength++;
  } //End While available
  // Serial.print(instruction[0]);
}

// Acknowledge pi communication requests
void I2Crequest() {
  Wire.write(talkToPi);
  reply = talkToPi;
}
