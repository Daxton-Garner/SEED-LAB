//Description: This code is what controls the wheel to turn to the specific angle requested using the 
//             motors current position and some PI controllers to manage the rise time and overshoot.
//Hardware:    You will need an Arduino Uno connected to a motor, a voltage monitor, and Raspberry Pi.
//             The motor should have a wheel attached and the Raspberry Pi will need a camera.


#include <Wire.h> //This is our Pi communication library
#define A2 2 //Left Motor Encoder
#define B2 5 //Left Motor Encoder
#define A1 3 //Right Motor Encoder
#define B1 6 //Right Motor Encoder
#define PWMPINL 9  //Left Motor PWM
#define PWMPINR 10  //Right Motor PWM
#define DIRPINL 7  //Left Motor Direction
#define DIRPINR 8  //Right Motor Direction
#define PIN4 4
#define MY_ADDR 8

bool debug = 0;
float Kp = 4; // 1.7 was Our gain we found in our insitiall simulink
float Kp_pos = 1; // The proportional gain
float Ki_pos = 2.5; // the intergral gain


float targetdistft = 7;  //Circumfrance of wheel 47.1 cm or 0.471 m
float targetdistm;
float wheelCir = 0.471;
float targetRad;
float desired_speed;
float integral_error = 0;
float pos_error = 0;
float error = 0;

int value;
int motor_dir = 1;
long encoderCountsL = 0;
long encoderCountsR = 0;
int lastA2, lastB2, lastA1, lastB1;
unsigned long desired_Ts_ms = 10;  // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
float last_time;

float ang_velocity;
float last_pos_rad;

void A2Change() {                            //Interrupt handler for pin A
  if (digitalRead(A2) == digitalRead(B2)) {  //Encoder direction check
    encoderCountsL -= 2;                      //Adjusts for changes made during periodic reporting
  } else {
    encoderCountsL += 2;
  }
  lastA2 = digitalRead(A2);  //Reset states
  lastB2 = digitalRead(B2);
}

void A1Change() {                            //Interrupt handler for pin A
  if (digitalRead(A1) == digitalRead(B1)) {  //Encoder direction check
    encoderCountsR -= 2;                      //Adjusts for changes made during periodic reporting
  } else {
    encoderCountsR += 2;
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

  targetdistm = 0.3048 * targetdistft;
  
  targetRad = (2*PI) * (targetdistm / wheelCir);
}

void loop() {
  float pos_rad;

  current_time = (float)(last_time_ms - start_time_ms) / 1000; //Current time (sec)

  pos_rad = 2 * PI * (float)encoderCountsL / 3200.0;  //Convert positions from steps to radians

  ang_velocity = (pos_rad - last_pos_rad) / (current_time - last_time); //Calculate angular velocity


  pos_error = targetRad - pos_rad;  //Calc position error
  integral_error = integral_error + pos_error * ((float)desired_Ts_ms / 1000); //Discrete integration (just a sum)
  Serial.println(integral_error);
  if(integral_error > 1.5) {integral_error = 1.5;}
  desired_speed = Kp_pos * pos_error + Ki_pos * integral_error; //Calc desired speed
  error = desired_speed - ang_velocity; //Velocity error
  value = Kp * error; //Final PWM value calculation

  if (value < 0) { //Translate negative pwm value to reversed motor direction
    value *= -1;
    motor_dir = 0;
  } else {
    motor_dir = 1;
  }
  
  value = value * 32; //Scale control loop output to PWM
  if (value > 100) { value = 100; } //Cap PI controller output at 255


  analogWrite(PWMPINL, value); //Control direction speed
  analogWrite(PWMPINR, value); //Control direction speed

  if (motor_dir) {
    digitalWrite(DIRPINL, LOW);
    digitalWrite(DIRPINR, HIGH);
  } else {
    digitalWrite(DIRPINL, HIGH);
    digitalWrite(DIRPINR, LOW);
  }

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
