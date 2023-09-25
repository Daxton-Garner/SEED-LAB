int pin10 = 10;
int pin9 = 9;
int pin8 = 8;
int pin7 = 7;
int pin4 = 4;

int value = 75;


void setup() {
  pinMode(pin10, OUTPUT);
  pinMode(pin9, OUTPUT);
  pinMode(pin8, OUTPUT);
  pinMode(pin7, OUTPUT);
  pinMode(pin4, OUTPUT);

  digitalWrite(pin4, HIGH);
}

void loop() {
  analogWrite(pin9, value);
  analogWrite(pin10, value);

  digitalWrite(pin7, HIGH);
  digitalWrite(pin8, LOW);
}
