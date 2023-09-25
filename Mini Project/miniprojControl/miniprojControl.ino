#define A2 2
#define B2 5
#define PWMPIN 9 //Motor PWM
#define DIRPIN 7 //Motor Direction
#define PIN4 4

int value = 75;
long encoderCounts = 0;
int lastA2, lastB2;
unsigned long desired_Ts_ms = 100;  // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

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
  
}

void loop() {

  float pos_rad;

  analogWrite(PWMPIN, value);

  digitalWrite(DIRPIN, HIGH);


  pos_rad = -2 * PI * (float)encoderCounts / 3200.0; //Convert positions from steps to radians

  Serial.print(current_time);
  Serial.print("\t");
  Serial.print(encoderCounts);
  Serial.print("\t");
  Serial.print(pos_rad);
  Serial.println(";");
  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
}
