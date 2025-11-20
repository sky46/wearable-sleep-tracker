void setup() {
  Serial.begin(38400);
  Serial2.begin(38400);
  while(!Serial);
  // Serial2.println("AT+BAUD6");
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
