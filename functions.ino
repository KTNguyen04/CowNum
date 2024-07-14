#define BUZZER_PIN 8
//buzzer
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

bool isButtonPressed() {
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH) {
    return true;
  } else {
    return false;
  }
}
//example main
void setup() {
  // Initialize the buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  // Initialize the button
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
  buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == HIGH) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
  delay(50);
}
