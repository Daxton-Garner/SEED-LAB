/*
Trevor Wolf
EENG350
9/18/2023
Assignment 2B
*/
#define PIN9 9
#define PIN10 10
#define A1 3
#define B1 6
#define A2 2
#define B2 5

unsigned long desired_Ts_ms = 100;  // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
int thisA1;                     //Current A Pin State
int thisB1;                     //Current B Pin State
int lastA1;                     //Previous A Pin State
int lastB1;                     //Previous B Pin State
int thisA2;                     //Current A Pin State
int thisB2;                     //Current B Pin State
int lastA2;                     //Previous A Pin State
int lastB2;                     //Previous B Pin State
long encoder1Counts = 0;        //Encoder count tracker
long encoder2Counts = 0;        //Encoder count tracker
unsigned long lastChange1 = 0;  //Milisecond count when last change occured, part of debounce
unsigned long lastChange2 = 0;  //Milisecond count when last change occured, part of debounce
int lastSingle1 = 0;            //Stores changes applied when count is adjusted prior to periodic reporting
int lastSingle2 = 0;            //Stores changes applied when count is adjusted prior to periodic reporting
float pos1_in_old;
float pos2_in_old;
float Xold;
float Yold;
float phi_old;

void A1Change() {  //Interrupt handler for pin A
  //if (millis() - lastChange1 > 10) {           //Debounce
  //  lastChange1 = millis();                    //Reset debounce
  if (digitalRead(A1) == digitalRead(B1)) {  //Encoder direction check
    encoder1Counts -= 2 - lastSingle1;       //Adjusts for changes made during periodic reporting
  } else {
    encoder1Counts += 2 - lastSingle1;
  }
  lastSingle1 = 0;  //Reset periodic reporting tracker
  //}
  lastA1 = digitalRead(A1);  //Reset states
  lastB1 = digitalRead(B1);

  //Serial.println(myEnc());
}

void A2Change() {  //Interrupt handler for pin A
  //if (millis() - lastChange2 > 10) {           //Debounce
  // lastChange2 = millis();                    //Reset debounce
  if (digitalRead(A2) == digitalRead(B2)) {  //Encoder direction check
    encoder2Counts += 2 - lastSingle2;       //Adjusts for changes made during periodic reporting
  } else {
    encoder2Counts -= 2 - lastSingle2;
  }
  lastSingle2 = 0;  //Reset periodic reporting tracker
  //}
  lastA2 = digitalRead(A2);  //Reset states
  lastB2 = digitalRead(B2);

  //Serial.println(myEnc());
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);     // Set the baud rate fast so that we can display the results
  last_time_ms = millis();  // set up sample time variable
  start_time_ms = last_time_ms;
  pinMode(A1, INPUT_PULLUP);  //Set inputs and pullups
  pinMode(B1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);  //Set inputs and pullups
  pinMode(B2, INPUT_PULLUP);
  pinMode(PIN9, OUTPUT);
  pinMode(PIN10, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(A1), A1Change, CHANGE);  // attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(A2), A2Change, CHANGE);  // attach interrupt handler
}

void loop() {
  // put your main code here, to run repeatedly:

  long pos1_counts;
  long pos2_counts;
  float pos1_rad;
  float pos2_rad;

  float pos1_in;
  float pos2_in;

  float Xpos;
  float Ypos;
  float phi;


  pos1_counts = myEnc(1);  // your encoder function that returns the current
  // position for motor 1
  pos2_counts = myEnc(2);  // your encoder function that returns the current
  // position for motor 2
  pos1_rad = 2 * PI * (float)pos1_counts / 3200.0; //Convert positions from steps to radians
  pos2_rad = 2 * PI * (float)pos2_counts / 3200.0;
  // print out timestamp in seconds
  current_time = (float)(last_time_ms - start_time_ms) / 1000.0;

  pos1_in = 3.0 * pos1_rad; //Convert radians to inches
  pos2_in = 3.0 * pos2_rad;
  Xpos = Xold + ((float)cos(phi_old)) * (pos1_in - pos1_in_old + pos2_in - pos2_in_old) * 0.5; //Calc X
  Ypos = Yold + ((float)sin(phi_old)) * (pos1_in - pos1_in_old + pos2_in - pos2_in_old) * 0.5; //Calc Y
  phi = phi_old + (1.0 / 14.25) * (pos1_in - pos1_in_old - (pos2_in - pos2_in_old)); //Calc Phi

//Update storage variables for next loop
  Xold = Xpos;
  Yold = Ypos;
  phi_old = phi;
  pos1_in_old = pos1_in;
  pos2_in_old = pos2_in;

//Output
  Serial.print(current_time);
  Serial.print("\t");
  Serial.print(Xpos);
  Serial.print("\t");
  Serial.print(Ypos);
  Serial.print("\t");
  Serial.print(phi);
  // Serial.print("\t");
  // Serial.print((pos1_in - pos1_in_old + pos2_in - pos2_in_old));
  Serial.println(";");
  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
}


int myEnc(int motor) {
  if (motor == 1) {
    //Updating within myEnc was causing numerous problems so the interrupt was deemed enough
    // thisA1 = digitalRead(A1);  //Get current states
    // thisB1 = digitalRead(B1);
    // if ((lastA1 != thisA1) || (lastB1 != thisB1)) {           //Check for update by 1
    //   lastSingle1 = -getDir(thisA1, thisB1, lastA1, lastB1);  //Store record of change for later correction
    //   encoder1Counts += lastSingle1;                          //Apply correction for output
    //   lastA1 = thisA1;                                        //Shift states
    //   lastB1 = thisB1;
    // }
    return encoder1Counts;
  } else {
    // thisA2 = digitalRead(A2);  //Get current states
    // thisB2 = digitalRead(B2);
    // if ((lastA2 != thisA2) || (lastB2 != thisB2)) {           //Check for update by 1
    //   lastSingle2 = -getDir(thisA2, thisB2, lastA2, lastB2);  //Store record of change for later correction
    //   encoder2Counts += lastSingle2;                          //Apply correction for output
    //   lastA2 = thisA2;                                        //Shift states
    //   lastB2 = thisB2;
    // }
    return encoder2Counts;
  }

  //Serial.println(outCounts);
}

int getDir(int pA, int pB, int cA, int cB) {
  if (pA == pB && cA == cB) {  //No change
    return 0;
  }
  int state = 1000 * pA + 100 * pB + 10 * cA + cB;  //Turns 4 bit states into 4 digit int for ease of switch/case readablility
  switch (state) {                                  //Returns increment direction based state
    case 0001:
      return 1;
      break;
    case 0010:
      return -1;
      break;
    case 0100:
      return -1;
      break;
    case 0111:
      return 1;
      break;
    case 1000:
      return 1;
      break;
    case 1011:
      return -1;
      break;
    case 1101:
      return -1;
      break;
    case 1110:
      return 1;
      break;
  }
}