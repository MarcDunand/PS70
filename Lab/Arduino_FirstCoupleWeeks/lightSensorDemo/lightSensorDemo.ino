int rPin = 23;

void setup() {
  Serial.begin(9600);
  pinMode(rPin, INPUT);
}

void loop() {
  int val = analogRead(rPin);
  Serial.println(val);
}