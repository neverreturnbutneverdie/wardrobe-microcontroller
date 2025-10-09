#define BLYNK_TEMPLATE_ID "TMPL6fWSOI-Lv"
#define BLYNK_TEMPLATE_NAME "Project End"
#define BLYNK_AUTH_TOKEN "xYLh8mJ4ltIYttb9G_0XRLSJ_9W36n4e"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>
#include <IRremote.hpp>
#include <UniversalTelegramBot.h>

// ==== ขากำหนดต่าง ๆ ====
// Ultrasonic ซ้าย
#define TRIG_LEFT  23
#define ECHO_LEFT  5
// Ultrasonic ขวา
#define TRIG_RIGHT 25
#define ECHO_RIGHT 26
// PIR Sensor สำหรับหยุดตอนปิด (เปลี่ยนจาก Ultrasonic Safety)
#define PIR_SAFE 16   

#define BUZZER_PIN 2
#define RELAY_IN1  22
#define RELAY_IN2  4

#define DHTPIN 21    
#define DHTTYPE DHT22
#define IR_RECV_PIN 27
#define SERVO_LEFT_PIN 17
#define SERVO_RIGHT_PIN 18  

// ==== L298N ==== 
#define IN1 32
#define IN2 33
#define IN3 15
#define IN4 19

const int DETECTION_DISTANCE = 5; // cm
const int SERVO_OPEN_US = 1000;
const int SERVO_CLOSE_US = 2000;
const int SERVO_STOP_US = 1500;

// ==== เวลามอเตอร์แยกซ้าย-ขวา ==== 
const int MOTOR_LEFT_OPEN_DURATION  = 14800; // ms (เดิม)
const int MOTOR_LEFT_CLOSE_DURATION = 13600; // ms (เดิม)
const int MOTOR_RIGHT_OPEN_DURATION  = 5500; // ms (ใหม่ 6.5 วิ)
const int MOTOR_RIGHT_CLOSE_DURATION = 5500; // ms (ใหม่ 6.5 วิ)

const unsigned long DHT_READ_INTERVAL = 2000;
const unsigned long IR_TIMEOUT = 200; // ms

// PIR Sensor timeout settings
const unsigned long PIR_CLEAR_TIMEOUT = 3000; // รอ 3 วินาที หลังจาก PIR หยุดตรวจจับ

char ssid[] = "Dis";
char pass[] = "88888888";

#define BOT_TOKEN "8298850961:AAE7ayqZyUMjiav7B2I9O7Bx-zzfiRHSEWU"
#define CHAT_ID   "7819961503"

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

const float TEMP_MIN = 20.0;
const float TEMP_MAX = 35.0;
const float HUM_MIN  = 60.0;
const float HUM_MAX  = 90.0;

DHT dht(DHTPIN, DHTTYPE); 
Ultrasonic ultrasonicLeft(TRIG_LEFT, ECHO_LEFT);
Ultrasonic ultrasonicRight(TRIG_RIGHT, ECHO_RIGHT);
Servo servoLeft, servoRight;

unsigned long leftServoStartTime = 0;
unsigned long rightServoStartTime = 0;
bool isLeftServoRunning = false, isRightServoRunning = false;
bool isLeftServoOpening = true, isRightServoOpening = true;
unsigned long leftRemainingTime = 0;
unsigned long rightRemainingTime = 0;

bool relayState = false;

// ==== แก้ relay IN1 ====
int relayIn1State = HIGH;   // เริ่มต้น HIGH = ปิด (สำหรับรีเลย์ Active LOW)

unsigned long lastUltrasonicTrigger = 0;
const unsigned long ULTRASONIC_DEBOUNCE = 1000;

unsigned long lastIRCommand = 0;
unsigned long lastIRReceiveTime = 0;
#define BTN_UP    0x18
#define BTN_DOWN  0x52

// ตัวแปรสำหรับจัดการ PIR sensor
unsigned long lastPIRTrigger = 0;
bool pirDetected = false;

void buzz(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
}

long readUltrasonic(Ultrasonic &sensor) {
  long distance = sensor.read(CM);
  if (distance <= 0 || distance > 5) return -1;
  return distance;
}

// เปลี่ยนฟังก์ชัน isSafeClear() เป็นใช้ PIR sensor พร้อม timeout
bool isSafeClear() {
  int pirState = digitalRead(PIR_SAFE);
  
  if (pirState == HIGH) {
    pirDetected = true;
    lastPIRTrigger = millis();
    Serial.println("[PIR] Motion detected - NOT SAFE");
    return false;
  } else {
    if (pirDetected) {
      if (millis() - lastPIRTrigger >= PIR_CLEAR_TIMEOUT) {
        pirDetected = false;
        Serial.println("[PIR] Area clear after timeout - SAFE");
        return true;
      } else {
        Serial.println("[PIR] Waiting for clear timeout - NOT SAFE");
        return false;
      }
    } else {
      return true;
    }
  }
}

// ==== Actuator ====
void actuator_up() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("[Actuator] UP");
}

void actuator_down() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("[Actuator] DOWN");
}

void actuator_stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("[Actuator] STOP");
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_IN1, OUTPUT);
  digitalWrite(RELAY_IN1, relayIn1State);   // ปิดรีเลย์ไว้ก่อน
  pinMode(RELAY_IN2, OUTPUT);
  digitalWrite(RELAY_IN2, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  actuator_stop();

  pinMode(PIR_SAFE, INPUT);

  dht.begin();
  servoLeft.attach(SERVO_LEFT_PIN, 500, 2500);
  servoRight.attach(SERVO_RIGHT_PIN, 500, 2500);
  servoLeft.writeMicroseconds(SERVO_STOP_US);
  servoRight.writeMicroseconds(SERVO_STOP_US);

  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);

  WiFi.begin(ssid, pass);
  secured_client.setInsecure();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();

  // ==== IR Remote ====
  if (IrReceiver.decode()) {
    lastIRCommand = IrReceiver.decodedIRData.command;
    lastIRReceiveTime = millis();
    IrReceiver.resume();
  }

  if (millis() - lastIRReceiveTime <= IR_TIMEOUT) {
    if (lastIRCommand == BTN_UP) actuator_up();
    else if (lastIRCommand == BTN_DOWN) actuator_down();
  } else {
    actuator_stop();
  }

  // ==== Ultrasonic ====
  long distanceLeft = readUltrasonic(ultrasonicLeft);
  long distanceRight = readUltrasonic(ultrasonicRight);

  bool detected = (distanceLeft > 0 && distanceLeft <= DETECTION_DISTANCE) ||
                  (distanceRight > 0 && distanceRight <= DETECTION_DISTANCE);

  if (detected && millis() - lastUltrasonicTrigger > ULTRASONIC_DEBOUNCE) {
    relayIn1State = (relayIn1State == HIGH ? LOW : HIGH);  // toggle
    digitalWrite(RELAY_IN1, relayIn1State);
    Serial.print("[Ultrasonic Trigger] Relay IN1 switched to: ");
    Serial.println(relayIn1State == LOW ? "ON (LOW)" : "OFF (HIGH)");
    lastUltrasonicTrigger = millis();
  }

  // ==== DHT ====
  static unsigned long lastDHTRead = 0;
  if (millis() - lastDHTRead > DHT_READ_INTERVAL) {
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    lastDHTRead = millis();
    if (!isnan(temp) && !isnan(hum)) {
      Blynk.virtualWrite(V0, temp);
      Blynk.virtualWrite(V1, hum);
      if (temp < TEMP_MIN || temp > TEMP_MAX)
        bot.sendMessage(CHAT_ID, "⚠️ อุณหภูมิผิดปกติ!\nTemp = " + String(temp) + "°C", "");
      if (hum < HUM_MIN || hum > HUM_MAX)
        bot.sendMessage(CHAT_ID, "⚠️ ความชื้นผิดปกติ!\nHumidity = " + String(hum) + "%", "");
    }
  }

  // ==== Servo LEFT ====
  if (!isLeftServoRunning && distanceLeft > 0 && distanceLeft <= DETECTION_DISTANCE) {
    Serial.println("[Trigger] LEFT Servo");
    buzz(2);
    leftServoStartTime = millis();
    isLeftServoRunning = true;
    servoLeft.writeMicroseconds(isLeftServoOpening ? SERVO_OPEN_US : SERVO_CLOSE_US);
    if (!isLeftServoOpening) leftRemainingTime = MOTOR_LEFT_CLOSE_DURATION;
  } else if (!isLeftServoOpening && !isLeftServoRunning && leftRemainingTime > 0 && isSafeClear()) {
    Serial.println("[Resume] LEFT Servo continue closing");
    leftServoStartTime = millis();
    isLeftServoRunning = true;
    servoLeft.writeMicroseconds(SERVO_CLOSE_US);
  } else if (isLeftServoRunning) {
    unsigned long elapsed = millis() - leftServoStartTime;
    if (!isLeftServoOpening && !isSafeClear()) {
      if (elapsed < leftRemainingTime) leftRemainingTime -= elapsed;
      else leftRemainingTime = 0;
      servoLeft.writeMicroseconds(SERVO_STOP_US);
      isLeftServoRunning = false;
      Serial.println("[Safety] LEFT stopped mid-close");
    } else if (elapsed >= (isLeftServoOpening ? MOTOR_LEFT_OPEN_DURATION : leftRemainingTime)) {
      servoLeft.writeMicroseconds(SERVO_STOP_US);
      isLeftServoRunning = false;
      if (isLeftServoOpening) isLeftServoOpening = false; // เปิดเสร็จค้าง
      else { isLeftServoOpening = true; leftRemainingTime = 0; } // ปิดเสร็จ
      Serial.println("[Stop] LEFT Servo");
    }
  }

  // ==== Servo RIGHT ====
  if (!isRightServoRunning && distanceRight > 0 && distanceRight <= DETECTION_DISTANCE) {
    Serial.println("[Trigger] RIGHT Servo");
    buzz(2);
    rightServoStartTime = millis();
    isRightServoRunning = true;
    servoRight.writeMicroseconds(isRightServoOpening ? SERVO_OPEN_US : SERVO_CLOSE_US);
    if (!isRightServoOpening) rightRemainingTime = MOTOR_RIGHT_CLOSE_DURATION;
  } else if (!isRightServoOpening && !isRightServoRunning && rightRemainingTime > 0 && isSafeClear()) {
    Serial.println("[Resume] RIGHT Servo continue closing");
    rightServoStartTime = millis();
    isRightServoRunning = true;
    servoRight.writeMicroseconds(SERVO_CLOSE_US);
  } else if (isRightServoRunning) {
    unsigned long elapsed = millis() - rightServoStartTime;
    if (!isRightServoOpening && !isSafeClear()) {
      if (elapsed < rightRemainingTime) rightRemainingTime -= elapsed;
      else rightRemainingTime = 0;
      servoRight.writeMicroseconds(SERVO_STOP_US);
      isRightServoRunning = false;
      Serial.println("[Safety] RIGHT stopped mid-close");
    } else if (elapsed >= (isRightServoOpening ? MOTOR_RIGHT_OPEN_DURATION : rightRemainingTime)) {
      servoRight.writeMicroseconds(SERVO_STOP_US);
      isRightServoRunning = false;
      if (isRightServoOpening) isRightServoOpening = false; // เปิดเสร็จค้าง
      else { isRightServoOpening = true; rightRemainingTime = 0; } // ปิดเสร็จ
      Serial.println("[Stop] RIGHT Servo");
    }
  }

  delay(50);
}

// ==== Blynk RELAY_IN2 ====
BLYNK_WRITE(V3) {
  int pinValue = param.asInt();
  relayState = (pinValue == 1);
  digitalWrite(RELAY_IN2, relayState ? HIGH : LOW);
  Serial.print("[Blynk Relay Control] RELAY_IN2 set to: ");
  Serial.println(relayState ? "ON" : "OFF");
  Blynk.virtualWrite(V3, relayState);
}