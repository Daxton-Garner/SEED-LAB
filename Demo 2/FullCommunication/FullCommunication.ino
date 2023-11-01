//Description: This code is what controls the wheel to turn to the specific angle requested using the
//             motors current position and some PI controllers to manage the rise time and overshoot.
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

bool debug = 0;
float Kp = 4;        // 1.7 was Our gain we found in our insitiall simulink
float Kp_pos = 1;    // The proportional gain
float Ki_pos = 2.5;  // the intergral gain

float readAngle;
float readDistance;
float lastReadAngle;
float lastReadDistance;
int currentState = 1;

float wheelCir = 0.471;
float targetrotm;
float targetRad;

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
float last_pos_rad[2];

char txBuf[7];
char rxBuf[7];
int rxInd = 0;
int reply = 0;

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

  attachInterrupt(digitalPinToInterrupt(A2), A2Change, CHANGE);  // attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(A1), A1Change, CHANGE);  // attach interrupt handler

  pinMode(PWMPINL, OUTPUT);
  pinMode(DIRPINL, OUTPUT);

  pinMode(PWMPINR, OUTPUT);
  pinMode(DIRPINR, OUTPUT);

  pinMode(PIN4, OUTPUT);
  digitalWrite(PIN4, HIGH);

  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(I2Creceive);
  // Send data back
  Wire.onRequest(I2Crequest);
}

void loop() {

  // Reading Angle and Distance from Marker

  if (rxInd > 0) {  //Check for incoming data
    // read the incoming byte:
    char incomingByte = rxBuf[0];  //Assumes only one character will be transmitted
    rxInd = 0;                     //Resets indication that buffer has data
    Serial.print("Read ");
    Serial.print(incomingByte);
    Serial.println(" from Pi");
    updateTarget(incomingByte);  //Update target location for PI controller
    if (incomingByte == 'd') {   //Toggle debug prints
      debug = !debug;
    } else if (incomingByte == 'S') {  //Reset timer and full throttle motor - for parameter tuning
      current_time = 0.0;
      last_time_ms = millis();  // reset sample time variable
      start_time_ms = last_time_ms;
      value = 100.0;
    } else if (incomingByte == '1' or incomingByte == '2' or incomingByte == '3' or incomingByte == '4') {
    }
  }

  if (currentState == 1) {
    digitalWrite(DIRPINL, LOW);   //Control direction
    digitalWrite(DIRPINR, HIGH);  //Control direction

    analogWrite(PWMPINL, 20);  //Control direction speed
    analogWrite(PWMPINR, 20);  //Control direction speed
  } else if (currentState == 2) {
    digitalWrite(DIRPINL, HIGH);  //Control direction
    digitalWrite(DIRPINR, HIGH);  //Control direction

    analogWrite(PWMPINL, 20);  //Control direction speed
    analogWrite(PWMPINR, 20);  //Control direction speed
  } else if (currentState == 3) {
    targetdeg = 90.0;
    targetrotm = (targetdeg / 360.0) * (0.685 * PI);
    targetRad = (2 * PI) * (targetrotm / wheelCir);
  } else {

    //Circle time
  }

  float pos_rad[2];

  current_time = (float)(last_time_ms - start_time_ms) / 1000;  //Current time (sec)

  if (currentState == 3) {

    for (int i = 0; i < 2; i++) {
      pos_rad[i] = 2 * PI * (float)encoderCounts[i] / 3200.0;  //Convert positions from steps to radians
      //if (!turnComplete && i == 1) { pos_rad[i] = pos_rad[i]*(-1);} //Make one wheel go backwards during turn

      ang_velocity[i] = (pos_rad[i] - last_pos_rad[i]) / (current_time - last_time);  //Calculate angular velocity
      if (!turnComplete && i == 1) {
        pos_error[i] = (-1) * targetRad - pos_rad[i];
      } else {
        pos_error[i] = targetRad - pos_rad[i];
      }

      integral_error[i] = integral_error[i] + pos_error[i] * ((float)desired_Ts_ms / 1000);  //Discrete integration (just a sum)
      //Serial.println(integral_error[i]);
      if (integral_error[i] > 1.5) { integral_error[i] = 1.5; }
      if (integral_error[i] < -1.5) { integral_error[i] = -1.5; }
      desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];  //Calc desired speed
      error[i] = desired_speed[i] - ang_velocity[i];                          //Velocity error
      value[i] = Kp * error[i];                                               //Final PWM value calculation

      if (value[i] < 0) {  //Translate negative pwm value to reversed motor direction
        value[i] *= -1;
        motor_dir[i] = 0;
      } else {
        motor_dir[i] = 1;
      }

      value[i] = value[i] * 32;                //Scale control loop output to PWM
      if (value[i] > 100) { value[i] = 100; }  //Cap PI controller output at 255
    }


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

  dtostrf(last_pos_rad, 3, 2, txBuf);  //Float to string for reporting back to pi -Defunct
  //Wire.write(txBuf);
  Serial.println(txBuf);  //Alt Debug

  if (debug) {
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(value[0]);
    Serial.print("\t");
    Serial.print(encoderCounts[0]);
    Serial.print("\t");
    Serial.print(value[1]);
    Serial.print("\t");
    Serial.print(encoderCounts[1]);
    Serial.println("");
  }

  last_pos_rad[0] = pos_rad[0];
  last_pos_rad[1] = pos_rad[1];

  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
  last_time = current_time;
}

//Translate the previous target and current target to a step target adjustment
void updateTarget(char tgtCmd) {
  lastTargetNum = targetNum;                   //
  targetNum = tgtCmd - '0';                    //Char to int
  if (targetNum == 1 && lastTargetNum == 4) {  // 4 to 1 transition
    targetCnts -= 800;
  } else if (targetNum == 4 && lastTargetNum == 1) {  // 1 to 4 transition
    targetCnts += 800;
  } else if (targetNum > lastTargetNum + 1) {  // 1 to 3 transition and 2 to 4 transition
    targetCnts -= 1600;
  } else if (targetNum < lastTargetNum - 1) {  //3 to 1 transition and 4 to 2 transition
    targetCnts += 1600;
  } else if (targetNum > lastTargetNum) {  //1 to 2, 2 to 3, 3 to 4 transitions
    targetCnts -= 800;
  } else if (targetNum < lastTargetNum) {  //4 to 3, 3 to 2, 2 to 1 transitions
    targetCnts += 800;
  }
  targetRad = 2 * PI * (float)targetCnts / 3200.0;  //Convert to radians for PI loop
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
