#define A2 2
#define B2 5
#define PWMPIN 9 //Motor PWM
#define DIRPIN 7 //Motor Direction
#define PIN4 4

int value;
long encoderCounts = 0;
int lastA2, lastB2;
unsigned long desired_Ts_ms = 10;  // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
float last_time;

float ang_velocity;
float last_pos_rad;

void A2Change() {  //Interrupt handler for pin A
  if (digitalRead(A2) == digitalRead(B2)) {  //Encoder direction check
    encoderCounts += 2;       //Adjusts for changes made during periodic reporting
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

  if (current_time < 1){
    value = 0;
  }else if(current_time > 1 && current_time < 3){
    value = 100.0;
  }else{
    value = 0;
  }

  analogWrite(PWMPIN, value);

  digitalWrite(DIRPIN, HIGH);

  current_time = (float)(last_time_ms - start_time_ms) / 1000;

  pos_rad = 2 * PI * (float)encoderCounts / 3200.0; //Convert positions from steps to radians

  ang_velocity = (pos_rad - last_pos_rad) / (current_time - last_time);

  if(current_time < 3.0){
  Serial.print(current_time);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("\t");
  Serial.print(ang_velocity);
  Serial.println("");
  }else{
    Serial.println("Finished");
  }

  last_pos_rad = pos_rad;

  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
  last_time = current_time;
}
