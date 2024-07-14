#define BUZZER_PIN 8
//buzzer
void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void buzzerOn() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerBeep(int duration) {
  buzzerOn();
  delay(duration);
  buzzerOff();
}

//button
#define BUTTON_PIN 7
int buttonState = 0;
void setupButton() {
  pinMode(BUTTON_PIN, INPUT);
}

bool isButtonPressed() {
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH) {
    return true;
  } else {
    return false;
  }
}
