
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <stdlib.h>

const char *ssid = "Wokwi-GUEST";
const char *password = "";
const char *mqttServer = "test.mosquitto.org";
const char *ID = "132201";
int connectCount = 0;
int port = 1883;

String confirmreset = "confirmreset";
String confirmreset2 = "confirmreset2";

WiFiClient espClient;
PubSubClient client(espClient);

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

const int buttonPin = 33;

const int n_lcd = 2;
LiquidCrystal_I2C lcd[n_lcd] = {
    LiquidCrystal_I2C(LCD1_ADDR, LCD_COLS, LCD_ROWS),
    LiquidCrystal_I2C(LCD2_ADDR, LCD_COLS, LCD_ROWS)};

int inCount = 0;
int outCount = 0;
int lastInCount = 0;
int lastOutCount = 0;

bool sendINOUTFlag = false;

bool lastOutUltrasonic = false;
bool lastInUltrasonic = false;

struct setting
{
  bool ledOn = true;
  bool buzzer = true;

  int ultrasonicDisMea = 40;
};
setting sett;

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
void mqttReconnect();
void sendINOUT();
void sendReset();

void callBack(char *topic, byte *message, unsigned int length);

void reset();

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
  Serial.begin(9600);

  for (int i = 0; i < n_ulso; i++)
  {
    pinMode(triggerPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  Serial.print("Connecting to WiFi");

  wifiConnect();

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttServer, port);
  client.setCallback(callBack);
}

void loop()
{
  if (!client.connected())
  {
    mqttReconnect();
    sendINOUT();
    connectCount++;
    Serial.println(connectCount);
  }
  client.loop();

  checkInUlso();
  if (sendINOUTFlag)
  {
    sendINOUT();
    sendINOUTFlag = false;
    Serial.println("\nSent.");
  }
  checkOutUlso();
  if (sendINOUTFlag)
  {
    sendINOUT();
    sendINOUTFlag = false;
    Serial.println("\nSent.");
  }
  updateSem();
  updateLCD();

  int resetSig = digitalRead(buttonPin);
  if (resetSig == 1)
  {
    sendReset();
    reset();
    resetSig = 0;
  }
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

    if (dis <= sett.ultrasonicDisMea)
    {
      lastInUltrasonic = true;
      turnLed(true);
    }
    if (!lastInUltrasonic)
      upSem0();
    if (lastInUltrasonic && lastOutUltrasonic)
    {

      inCount++;
      sendINOUTFlag = true;
      lastOutUltrasonic = false;
      lastInUltrasonic = false;
      turnLed(false);
      if (sett.buzzer)
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

    if (dis <= sett.ultrasonicDisMea)
    {
      lastOutUltrasonic = true;
      turnLed(true);
    }
    if (!lastOutUltrasonic)
      upSem1();
    if (lastInUltrasonic && lastOutUltrasonic)
    {

      outCount++;
      sendINOUTFlag = true;
      lastOutUltrasonic = false;
      lastInUltrasonic = false;
      turnLed(false);
      if (sett.buzzer)

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

    if (dis1 > sett.ultrasonicDisMea && dis0 > sett.ultrasonicDisMea)
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

  digitalWrite(ledPin, on && sett.ledOn);
}
void onBuzzer(int toneType)
{
  // ledcWriteTone(buzzerPin, toneType);
  tone(buzzerPin, toneType, buzzerDuration);
}

void wifiConnect()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void mqttReconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection... ");
    if (client.connect(ID))
    {
      Serial.println(" connected");
      client.subscribe("reset");
      client.subscribe("setting/led");
      client.subscribe("setting/buzzer");
      client.subscribe("setting/range");
    }
    else
    {
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void sendINOUT()
{
  char buffer[50];
  sprintf(buffer, "{\"inC\":%d,\"outC\":%d}", inCount, outCount);
  client.publish("stat", buffer);
}
void sendReset()
{
  char buffer[50];
  sprintf(buffer, confirmreset2.c_str());
  client.publish("reset", buffer);
}

void callBack(char *topic, byte *message, unsigned int length)
{
  Serial.print("topic: ");
  Serial.print(topic);
  String stMessage;
  for (int i = 0; i < length; ++i)
  {
    stMessage += (char)message[i];
  }
  if (strcmp(topic, "reset") == 0)
  {
    if (stMessage == confirmreset)
    {
      Serial.println(stMessage);
      reset();
      sendINOUTFlag = true;
    }
  }

  if (strcmp(topic, "setting/led") == 0)
  {
    Serial.println(stMessage);
    if (stMessage == "true")
      sett.ledOn = true;
    else
      sett.ledOn = false;
  }
  if (strcmp(topic, "setting/buzzer") == 0)
  {
    Serial.println(stMessage);
    if (stMessage == "true")
      sett.buzzer = true;
    else
      sett.buzzer = false;
  }
  if (strcmp(topic, "setting/range") == 0)
  {
    Serial.println(stMessage);
    sett.ultrasonicDisMea = atoi(stMessage.c_str());
  }
}

void reset()
{
  inCount = outCount = 0;
}
