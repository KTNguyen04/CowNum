
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

enum
{
  countingMode = 1,
  warningMode
};
bool ledOn = true;
bool buzzerOn = true;

#define LCD1_ADDR 0x27
#define LCD2_ADDR 0x28
#define toneIn 300
#define toneOut 200

int ultrasonicDisMea = 20; // 40cm
int maxUltrasonicDis = 400;

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

const int n_ulso = 2;
const int triggerPin[n_ulso] = {18, 4};
const int echoPin[n_ulso] = {19, 5};

const int ledPin = 26;

const int buzzerPin = 25;
const int buzzerDuration = 200;

const int n_lcd = 2;
LiquidCrystal_I2C lcd[n_lcd] = {
    LiquidCrystal_I2C(LCD1_ADDR, LCD_COLS, LCD_ROWS),
    LiquidCrystal_I2C(LCD2_ADDR, LCD_COLS, LCD_ROWS)};

int inCount = 0;
int outCount = 0;
int lastInCount = 0;
int lastOutCount = 0;

bool lastOutUltrasonic = false;
bool lastInUltrasonic = false;

int mySemaphore1 = 0;
int mySemaphore0 = 0;

void lcdDisplay(int lcd_num, int row, int col, String s);
void updateLCD();
void checkOutUlso();
void checkInUlso();

void upSem0();
void downSem0();
void upSem1();
void downSem1();
void updateSem();

void turnLed(bool on = true);
void onBuzzer(int toneType);

void wifiConnect();

void setup()
{
  for (int i = 0; i < n_lcd; i++)
  {
    lcd[i].init();
    lcd[i].backlight();
  }
  lcdDisplay(0, 0, 7, "IN");
  lcdDisplay(1, 0, 7, "OUT");
  updateLCD();
  Serial.begin(115200);

  for (int i = 0; i < n_ulso; i++)
  {
    pinMode(triggerPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  checkInUlso();
  checkOutUlso();
  updateSem();
  updateLCD();
  // Serial.println(mySemaphore0, DEC);
  // Serial.println(mySemaphore1, DEC);
}

void lcdDisplay(int lcd_num, int row, int col, String s)
{
  lcd[lcd_num].setCursor(col, row);
  lcd[lcd_num].print(s);
}
void updateLCD()
{
  lcdDisplay(0, 1, 7, String(inCount));
  lcdDisplay(1, 1, 7, String(outCount));
}

int readUltrasonicDistance(int triggerP, int echoP)
{

  digitalWrite(triggerP, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerP, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerP, LOW);
  return pulseIn(echoP, HIGH) * 0.01723;
}

void checkInUlso()
{
  if (mySemaphore0 >= 0)
  {
    downSem0();
    int dis = readUltrasonicDistance(triggerPin[0], echoPin[0]);

    if (dis <= ultrasonicDisMea)
    {
      lastInUltrasonic = true;
      turnLed(true);
    }
    if (!lastInUltrasonic)
      upSem0();
    if (lastInUltrasonic && lastOutUltrasonic)
    {

      inCount++;
      lastOutUltrasonic = false;
      lastInUltrasonic = false;
      turnLed(false);
      onBuzzer(toneIn);
    }
  }
}
void checkOutUlso()
{
  if (mySemaphore1 >= 0)
  {
    downSem1();
    int dis = readUltrasonicDistance(triggerPin[1], echoPin[1]);

    if (dis <= ultrasonicDisMea)
    {
      lastOutUltrasonic = true;
      turnLed(true);
    }
    if (!lastOutUltrasonic)
      upSem1();
    if (lastInUltrasonic && lastOutUltrasonic)
    {

      outCount++;
      lastOutUltrasonic = false;
      lastInUltrasonic = false;
      turnLed(false);
      onBuzzer(toneOut);
    }
  }
}
void updateSem()
{
  if (lastInCount == inCount && lastOutCount == outCount)
    return;
  if (lastInCount != inCount)
  {
    lastInCount = inCount;
  }
  if (lastOutCount != outCount)
  {
    lastOutCount = outCount;
  }
  while (true)
  {
    int dis1 = readUltrasonicDistance(triggerPin[1], echoPin[1]);
    int dis0 = readUltrasonicDistance(triggerPin[0], echoPin[0]);

    if (dis1 > ultrasonicDisMea && dis0 > ultrasonicDisMea)
    {
      upSem0();
      upSem1();
      break;
    }
  }
}
void downSem0()
{
  mySemaphore0--;
}
void upSem0()
{
  mySemaphore0++;
}
void downSem1()
{
  mySemaphore1--;
}
void upSem1()
{
  mySemaphore1++;
}
void turnLed(bool on)
{
  digitalWrite(ledPin, on && ledOn);
}
void onBuzzer(int toneType)
{
  //ledcWriteTone(buzzerPin, toneType);
 tone(buzzerPin, toneType, buzzerDuration);
}
