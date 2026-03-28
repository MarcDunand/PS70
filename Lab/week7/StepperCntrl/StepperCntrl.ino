const int stepPin = 13;
const int dirPin = 12;

void setup() {
  // put your setup code here, to run once:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(dirPin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(stepPin, LOW);
  delay(50);
  digitalWrite(stepPin, HIGH);
  delay(50);
}