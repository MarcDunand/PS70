const int A1A = D0;  // define pin 12 for A-1A 
const int A1B = D1;  // define pin 14 for A-1B 

void setup() {
  pinMode(A1A, OUTPUT);     // specify these pins as outputs
  pinMode(A1B, OUTPUT);
  digitalWrite(A1A, LOW);   // start with the motors off 
  digitalWrite(A1B, LOW);
}

void loop() {
  // start the motor 
  digitalWrite(A1A, HIGH);   
  digitalWrite(A1B, LOW);
  delay(4000);              // allow the motor to run for 4 seconds

  // stop the motor
  digitalWrite(A1A, LOW);   // setting both pins LOW stops the motor 
  digitalWrite(A1B, LOW);   // redundant, but doesn't hurt 
  delay(2000);              // keep the motor off for 2 seconds

}