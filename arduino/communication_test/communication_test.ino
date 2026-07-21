void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ARDUINO_READY");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "PING") {
      Serial.println("PONG");
    } else {
      Serial.println("UNKNOWN_COMMAND");
    }
  }
}