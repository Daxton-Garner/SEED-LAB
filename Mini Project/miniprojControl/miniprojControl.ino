#define A2 2
#define B2 5
#define PWMPIN 9  //Motor PWM
#define DIRPIN 7  //Motor Direction
#define PIN4 4


/*
1   2

4   3

1 -> 0
2 -> pi/2
3 -> pi
4 -> 3pi/2

*/
float Kp = 60;
float Kp_pos = 4;
float Ki_pos = 6;

int targetNum = 1;
int lastTargetNum;
int targetCnts;
float targetRad;
float desired_speed;
float integral_error = 0;
float pos_error = 0;
float error = 0 ;

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
}

void loop() {

  float pos_rad;

  if (Serial.available() > 0) {
    // read the incoming byte:
    char incomingByte = Serial.read();
    if (incomingByte == 'S') {
      current_time = 0.0;
      last_time_ms = millis();  // reset sample time variable
      start_time_ms = last_time_ms;
      value = 100.0;
    } else if (incomingByte == '1' or incomingByte == '2' or incomingByte == '3' or incomingByte == '4') {
      updateTarget(incomingByte);
    }
  }



  current_time = (float)(last_time_ms - start_time_ms) / 1000;

  pos_rad = 2 * PI * (float)encoderCounts / 3200.0;  //Convert positions from steps to radians

  ang_velocity = (pos_rad - last_pos_rad) / (current_time - last_time);


  pos_error = targetRad - pos_rad;
  integral_error = integral_error + pos_error * ((float)desired_Ts_ms / 1000);
  desired_speed = Kp_pos * pos_error + Ki_pos * integral_error;
  error = desired_speed - ang_velocity;
  value = Kp * error;

  if (value < 0) {
    value *= -1;
    motor_dir = 0;
  } else {
    motor_dir = 1;
  }
  if (value > 255) {value = 255;}

  analogWrite(PWMPIN, value);
  if (motor_dir) {
    digitalWrite(DIRPIN, HIGH);
  } else {
    digitalWrite(DIRPIN, LOW);
  }

  Serial.print(current_time);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("\t");
  Serial.print(ang_velocity);
  Serial.println("");

  last_pos_rad = pos_rad;

  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
  last_time = current_time;
}

void updateTarget(char tgtCmd) {
  lastTargetNum = targetNum;
  targetNum = tgtCmd - '0';
  if (targetNum == 1 && lastTargetNum == 4) {
    targetCnts += 800;
  } else if (targetNum == 4 && lastTargetNum == 1) {
    targetCnts -= 800;
  } else if (targetNum > lastTargetNum + 1) {
    targetCnts += 1600;
  } else if (targetNum < lastTargetNum - 1) {
    targetCnts -= 1600;
  }else if (targetNum > lastTargetNum) {
    targetCnts += 800;
  } else if (targetNum < lastTargetNum) {
    targetCnts -= 800;
  }
  targetRad = 2 * PI * (float)targetCnts / 3200.0;
}
