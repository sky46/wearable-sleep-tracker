void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  while(!Serial);
  Serial2.println("AT+BAUD4");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      Serial2.println();
    } else {
      Serial2.print(c);
    }
  }
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.print(c);
  }
}
