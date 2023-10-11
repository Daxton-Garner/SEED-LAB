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
bool turnComplete = 0;
int stallCount = 0;
float Kp = 4;        // 1.7 was Our gain we found in our insitiall simulink
float Kp_pos = 1;    // The proportional gain
float Ki_pos = 2.5;  // the intergral gain

float targetdeg = 270.0;
float targetdistft = 3;  //Circumfrance of wheel 47.1 cm or 0.471 m

float targetdistm;
//float targetrotrad = targetdeg * 3.14159 / 180.0;
float targetrotm = (targetdeg / 360.0) * (0.685 * PI);  //Wheel dist is 27.1cm aka 0.271m. Used to be a 0.5 here but I dropped it becuase it did exactly half what I asked

float wheelCir = 0.471;
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

  targetRad = (2 * PI) * (targetrotm / wheelCir);
}

void loop() {
  float pos_rad[2];

  

  current_time = (float)(last_time_ms - start_time_ms) / 1000;  //Current time (sec)

  for (int i = 0; i < 2; i++) {
    pos_rad[i] = 2 * PI * (float)encoderCounts[i] / 3200.0;     //Convert positions from steps to radians
    //if (!turnComplete && i == 1) { pos_rad[i] = pos_rad[i]*(-1);} //Make one wheel go backwards during turn

    ang_velocity[i] = (pos_rad[i] - last_pos_rad[i]) / (current_time - last_time);  //Calculate angular velocity
    if (!turnComplete && i == 1) { pos_error[i] = (-1)*targetRad - pos_rad[i];   }
    else { pos_error[i] = targetRad - pos_rad[i]; }

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

  if (ang_velocity[0] + ang_velocity[1] < 0.01) {
    stallCount++;
  }
  else {
    stallCount = 0;
  }
  if ((stallCount >= 500 || targetdeg == 0)&& !turnComplete) {
    Serial.println("Turn Complete");
    turnComplete = 1;
    encoderCounts[0] = 0;
    encoderCounts[1] = 0;
    integral_error[0] = 0;
    integral_error[1] = 0;
    pos_rad[0] = 0;
    pos_rad[1] = 0;
    targetdistm = 0.3048 * targetdistft;
    targetRad = (2 * PI) * (targetdistm / wheelCir);

  }

  last_pos_rad[0] = pos_rad[0];
  last_pos_rad[1] = pos_rad[1];

  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
  last_time = current_time;
}
