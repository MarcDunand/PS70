void setup() {
    Serial.begin(9600);
    pinMode(8, OUTPUT);
    pinMode(7, INPUT);
    digitalWrite(8, HIGH);
}

void loop() {
    Serial.println(digitalRead(7));

    delay(100);
}