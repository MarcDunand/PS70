//LED digitally controlled from a button
const int buttonPin = 13;
const int ledPin = 14;
int val;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  //Serial.println(digitalRead(buttonPin));
  val = digitalRead(buttonPin);
  if(val == LOW) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
}



//LED digitally controlled by potentiometer
/*const int ledPin = 14;
const int potPin = 27;
int ledVal;
int potVal;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(potPin, INPUT);
}

void loop() {
  potVal = analogRead(potPin);
  Serial.println(potVal);
  ledVal = map(potVal, 0, 4095, 0, 255);
  analogWrite(ledPin, ledVal);
}
*/