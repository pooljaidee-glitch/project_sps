// ============================================================
//  SMART PARKING ZONE A  (ESP32)  -- เวอร์ชันคอมเมนต์ทุกบรรทัด
//  ตรรกะเหมือน smart_parking_fixed.ino ทุกประการ
//  บรรทัดที่มีป้าย [แก้ไข] คือจุดที่แก้เพื่อกันรถทางออกติด
// ============================================================

#include <Wire.h>                       // ไลบรารีสื่อสาร I2C (ใช้กับจอ LCD)
#include <LiquidCrystal_I2C.h>          // ไลบรารีจอ LCD แบบ I2C
#include <ESP32Servo.h>                 // ไลบรารีควบคุมเซอร์โวบน ESP32
#include <WiFi.h>                       // ไลบรารี WiFi
#include <PubSubClient.h>               // ไลบรารี MQTT (ติดตั้ง "PubSubClient" โดย Nick O'Leary)
#include <Preferences.h>                // เก็บค่าลงหน่วยความจำถาวร (ไฟดับแล้วไม่หาย)
#include <time.h>                       // เวลาจริงจาก NTP

// ===== ตั้งค่า WiFi / MQTT =====
const char* WIFI_SSID = "First_2.4GHz"; // ชื่อ WiFi
const char* WIFI_PASS = "0852400955";   // รหัส WiFi

const char* MQTT_HOST = "broker.hivemq.com"; // ที่อยู่ MQTT broker
const int   MQTT_PORT = 1883;           // พอร์ต MQTT (ไม่เข้ารหัส)

const char* BASE = "project_parking_8392"; // Topic หลัก ต้องตรงกับบนเว็บ
const char* ZONE = "A";                 // ชื่อโซน ต้องตรงกับบนเว็บ

char TOPIC_ZONE[64], TOPIC_IN[64], TOPIC_OUT[64], TOPIC_LCD[64]; // ที่เก็บชื่อ topic ที่ประกอบแล้ว
WiFiClient wifiClient;                  // ตัวเชื่อมต่อ TCP ผ่าน WiFi
PubSubClient mqtt(wifiClient);          // ตัวไคลเอนต์ MQTT ที่ใช้ wifiClient

// ===== ขาอุปกรณ์ =====
#define SDA_PIN 21                      // ขา SDA ของ I2C
#define SCL_PIN 22                      // ขา SCL ของ I2C
#define SERVO_IN_PIN 18                 // ขาเซอร์โวไม้กั้นทางเข้า
#define SERVO_OUT_PIN 19                // ขาเซอร์โวไม้กั้นทางออก
#define IR_IN_1 34                      // เซนเซอร์ S1 หน้าไม้กั้นทางเข้า
#define IR_IN_2 35                      // เซนเซอร์ S2 หลังไม้กั้นทางเข้า
#define IR_EXIT 27                      // เซนเซอร์หน้าไม้กั้นทางออก
const int slotPins[4] = {32, 33, 25, 26}; // ขาเซนเซอร์ของช่องจอด A1-A4
const int TOTAL_SLOTS = 4;              // [แก้ไข] จำนวนช่องจอดทั้งหมด (ค่าคงที่)

#define DETECTED LOW                    // เซนเซอร์ให้ LOW เมื่อเจอวัตถุ

LiquidCrystal_I2C lcd(0x27, 20, 4);     // จอ LCD ที่อยู่ 0x27 ขนาด 20 ตัวอักษร x 4 บรรทัด
Servo entryServo;                       // ออบเจกต์เซอร์โวทางเข้า
Servo exitServo;                        // ออบเจกต์เซอร์โวทางออก

const int GATE_UP = 90;                 // มุมไม้กั้นตอนเปิด (องศา)
const int GATE_DOWN = 0;                // มุมไม้กั้นตอนปิด (องศา)
int entryAngle = GATE_DOWN;             // มุมปัจจุบันของไม้กั้นเข้า (ส่งขึ้นเว็บ)
int exitAngle = GATE_DOWN;              // มุมปัจจุบันของไม้กั้นออก (ส่งขึ้นเว็บ)

enum EntryState { ENT_IDLE, ENT_WAIT_PASS, ENT_CLEARING }; // สถานะทางเข้า: ว่าง / รอรถผ่าน / รอเคลียร์
EntryState entState = ENT_IDLE;         // สถานะทางเข้าตอนนี้
unsigned long entTimer = 0;             // เวลาที่ไม้กั้นเข้าเริ่มเปิด
unsigned long tailGateTimer = 0;        // เวลาที่ S1 และ S2 เจอรถพร้อมกันเริ่มต้น

enum ExitState { EX_IDLE, EX_OPEN };    // สถานะทางออก: ว่าง / ไม้กั้นเปิด
ExitState exitState = EX_IDLE;          // สถานะทางออกตอนนี้
unsigned long exitTimer = 0;            // เวลาล่าสุดที่เซนเซอร์ทางออกเจอรถ
unsigned long exitDetectSince = 0;      // เวลาที่เซนเซอร์ทางออกเริ่มเจอรถต่อเนื่อง

int passCount = 0;                      // จำนวนรถในลานตอนนี้ (นับสุทธิ)
unsigned long passIn = 0;               // รถเข้าสะสม (ส่งขึ้นเว็บ)
unsigned long passOut = 0;              // รถออกสะสม (ส่งขึ้นเว็บ)

const unsigned long S1_HOLD_MS = 700;   // S1 ต้องเจอรถต่อเนื่องเท่านี้ ไม้กั้นถึงเปิด
const unsigned long EXIT_HOLD_MS = 0;   // ทางออก: 0 = เจอรถแล้วยกทันที
const unsigned long CAR_MIN_MS = 120;   // S2 เจอวัตถุอย่างน้อยเท่านี้ถึงนับเป็นรถ
const unsigned long S2_GAP_MS = 60;     // ช่องว่างสัญญาณ S2 สั้นกว่านี้ถือเป็นรถคันเดิม
const unsigned long S1_TO_S2_MS = 20000; // ต้องเคยเจอที่ S1 ภายในเวลานี้ก่อนนับที่ S2
const unsigned long ENTRY_S2_DELAY_MS = 3000; // [แก้ไข] หลังยกไม้กั้นเข้า เมิน S2 นานเท่านี้ (3 วินาที)
bool s1Lock = false;                    // true = ไม้กั้นเปิดแล้วไม่มีรถผ่าน ห้ามเปิดซ้ำจนกว่า S1 ว่าง
unsigned long s1DetectSince = 0;        // เวลาที่ S1 เริ่มเจอรถต่อเนื่อง
unsigned long lastS1At = 0;             // เวลาล่าสุดที่ S1 เจอรถ
bool s2Armed = true;                    // พร้อมนับรถคันใหม่หรือยัง
unsigned long s2DetectSince = 0;        // (สำรอง) เวลาที่ S2 เริ่มเจอรถ
unsigned long s2ClearSince = 0;         // เวลาที่ S2 เริ่มว่างต่อเนื่อง
const unsigned long S2_DEBOUNCE_MS = 50; // (สำรอง) ค่ากันสัญญาณรบกวนของ S2
const unsigned long S2_REARM_MS = 300;  // S2 ต้องว่างต่อเนื่องเท่านี้ถึงนับคันต่อไปได้
bool countSynced = false;               // ซิงค์ passCount กับช่องจริงตอนเปิดเครื่องแล้วหรือยัง
unsigned long mismatchSince = 0;        // เวลาที่เริ่มพบช่องมีรถมากกว่าตัวนับ
const unsigned long MISMATCH_CONFIRM_MS = 5000; // ต้องผิดปกติต่อเนื่อง 5 วินาทีถึงแจ้งเตือน

unsigned long overCountSince = 0;       // [แก้ไข] เวลาที่เริ่มพบว่าตัวนับ มากกว่า ช่องที่มีรถจริง
const unsigned long COUNT_RESYNC_MS = 60000; // [แก้ไข] ผิดปกติเกินเท่านี้ (60 วินาที) ถึงรีเซ็ตตัวนับ

unsigned long occAt[4] = {0, 0, 0, 0};  // เวลาที่แต่ละช่องเริ่มมีรถ
String problemMask = "0000";            // ช่องที่มีปัญหา (1 = มีปัญหา) ส่งขึ้นเว็บ
bool straddle = false;                  // สถานะจอดคร่อมช่อง
unsigned long timeoutAlertUntil = 0;    // แสดงแจ้งเตือน ENTRY TIMEOUT ไปถึงเวลานี้

// ===== กรองสัญญาณเซนเซอร์ช่องจอด =====
const unsigned long SLOT_OCC_CONFIRM_MS  = 300;  // ว่าง -> มีรถ ต้องเจอต่อเนื่องเท่านี้
const unsigned long SLOT_FREE_CONFIRM_MS = 2000; // มีรถ -> ว่าง ต้องว่างต่อเนื่องเท่านี้
bool slotRaw[4]    = {false, false, false, false}; // ค่าดิบล่าสุดของเซนเซอร์แต่ละช่อง
bool slotStable[4] = {false, false, false, false}; // ค่าที่กรองแล้ว (ใช้ทั้งระบบ)
unsigned long slotRawSince[4] = {0, 0, 0, 0};      // เวลาที่ค่าดิบเปลี่ยนล่าสุด
bool slotInit = false;                  // อ่านช่องจอดรอบแรกแล้วหรือยัง

String prevSlots = "";                  // สถานะช่องรอบก่อน (ว่าง = ให้รอบแรกวาดจอเสมอ)
String welcomeMsg = String("** WELCOME ZONE ") + ZONE + " **"; // ข้อความต้อนรับ 20 ตัวอักษร
String alertMsg = welcomeMsg;           // ข้อความสถานะที่จะโชว์บรรทัดแรกของ LCD
String shownSig = "";                   // ลายเซ็นของสิ่งที่แสดงบนจออยู่ (เทียบเพื่อดูว่าต้องวาดใหม่ไหม)
bool blinkStatus = false;               // false = โชว์ชื่อระบบ, true = โชว์ข้อความสถานะ
unsigned long lastBlink = 0;            // เวลาที่สลับบรรทัดแรกล่าสุด
const unsigned long BLINK_MS = 2000;    // สลับทุก 2 วินาที
String lcdLines[4];                     // ข้อความ 4 บรรทัดของ LCD

String lastZone, lastIn, lastOut, lastLcd; // payload ล่าสุดที่ส่งไปแต่ละ topic (เทียบว่าเปลี่ยนไหม)
unsigned long lastHeartbeat = 0, lastMqttTry = 0, lastWifiTry = 0, lastDiag = 0; // ตัวจับเวลาต่างๆ
const unsigned long HEARTBEAT_MS = 100; // ส่งข้อมูลซ้ำทุก 100 ms เพื่อบอกว่าบอร์ดยังอยู่

// ===== ตัวนับเวลาจอดของแต่ละช่อง =====
Preferences prefs;                      // ออบเจกต์เก็บค่าถาวร
uint32_t slotSince[4] = {0, 0, 0, 0};   // เวลา epoch ที่รถเริ่มจอด (0 = ว่าง)
unsigned long slotFreeAt[4] = {0, 0, 0, 0}; // millis ที่ช่องเริ่มว่าง (กันสัญญาณกระตุก)
const unsigned long FREE_CONFIRM_MS = 10000; // ต้องว่างต่อเนื่อง 10 วินาทีถึงถือว่ารถออกจริง
const bool CLEAR_TIMERS_ON_BOOT = true; // เปิดเครื่องแล้วล้างเวลาจอดเก่าทิ้ง
bool bootSlotsChecked = false;          // ตรวจเวลาจอดค้างตอนบูตแล้วหรือยัง

uint32_t nowEpoch() {                   // ฟังก์ชันอ่านเวลาปัจจุบันเป็น epoch
  time_t t = time(nullptr);             // อ่านเวลาจากนาฬิการะบบ
  return (t > 1700000000) ? (uint32_t)t : 0; // ถ้าเวลาสมเหตุสมผล (NTP ซิงค์แล้ว) คืนค่า ไม่งั้นคืน 0
}

void saveSlot(int i) {                  // ฟังก์ชันบันทึกเวลาจอดของช่อง i ลงหน่วยความจำ
  char key[4];                          // ที่เก็บชื่อคีย์
  snprintf(key, sizeof(key), "s%d", i); // สร้างชื่อคีย์ เช่น s0, s1
  prefs.putUInt(key, slotSince[i]);     // เขียนค่าลงหน่วยความจำ
}

void updateSlotTimers(const String& slots) { // ฟังก์ชันอัปเดตตัวจับเวลาจอดของทุกช่อง
  uint32_t now = nowEpoch();            // เวลาปัจจุบัน
  if (!bootSlotsChecked) {              // ถ้ายังไม่ได้ตรวจตอนบูต
    bootSlotsChecked = true;            // ทำเครื่องหมายว่าตรวจแล้ว
    for (int i = 0; i < 4; i++) {       // วนทุกช่อง
      if (slots[i] != '1' && slotSince[i] != 0) { slotSince[i] = 0; saveSlot(i); } // ช่องว่างแต่มีเวลาค้าง -> ล้าง
    }
  }
  for (int i = 0; i < 4; i++) {         // วนทุกช่อง
    if (slots[i] == '1') {              // ถ้าช่องนี้มีรถ
      slotFreeAt[i] = 0;                // ยกเลิกการนับว่าง
      if (slotSince[i] == 0 && now != 0) { // ถ้ายังไม่มีเวลาเริ่ม และมีเวลาจริงแล้ว
        slotSince[i] = now;             // บันทึกเวลาเริ่มจอด
        saveSlot(i);                    // เก็บลงหน่วยความจำถาวร
      }
    } else if (slotSince[i] != 0) {     // ช่องว่างแต่ยังมีเวลาจอดค้างอยู่
      if (slotFreeAt[i] == 0) slotFreeAt[i] = millis(); // เริ่มนับเวลาว่าง
      if (millis() - slotFreeAt[i] > FREE_CONFIRM_MS) { // ว่างนานพอ = รถออกจริง
        slotSince[i] = 0;               // ล้างเวลาจอด
        slotFreeAt[i] = 0;              // ล้างตัวนับว่าง
        saveSlot(i);                    // บันทึกลงหน่วยความจำ
      }
    }
  }
}

// ===== จอ LCD =====
void buildLcdLines(int freeSlots, const String& slots) { // สร้างข้อความ 4 บรรทัดของจอ
  lcdLines[0] = blinkStatus ? alertMsg : String("=== SMART PARKING ==="); // บรรทัด 1 สลับระหว่างสถานะกับชื่อระบบ
  String l1 = "Free Slots: " + String(freeSlots) + "/4"; // บรรทัด 2 แสดงจำนวนช่องว่าง
  while (l1.length() < 20) l1 += ' ';   // เติมช่องว่างให้ครบ 20 ตัวอักษร
  lcdLines[1] = l1;                     // เก็บบรรทัด 2
  String freeList = "";                 // รายชื่อช่องว่าง
  int shown = 0;                        // จำนวนช่องที่แสดงชื่อแล้ว
  for (int i = 0; i < 4; i++) {         // วนทุกช่อง
    if (slots[i] == '0') {              // ถ้าช่องว่าง
      String n = String(ZONE) + String(i + 1); // ชื่อช่อง เช่น A1
      if (freeList.length() + n.length() + 1 > 17) break; // ถ้าเกินความกว้างจอให้หยุด
      freeList += n + " ";              // ต่อชื่อช่องเข้ารายการ
      shown++;                          // นับว่าแสดงแล้ว
    }
  }
  if (shown < freeSlots) freeList += "+" + String(freeSlots - shown); // ถ้าแสดงไม่หมด ใส่ +จำนวนที่เหลือ
  if (freeSlots == 0) freeList = "-";   // ไม่มีช่องว่างแสดงขีด
  String l2 = freeList;                 // บรรทัด 3
  while (l2.length() < 20) l2 += ' ';   // เติมช่องว่างให้ครบ 20 ตัวอักษร
  lcdLines[2] = l2;                     // เก็บบรรทัด 3
  lcdLines[3] = "                    "; // บรรทัด 4 ว่าง
}

void drawLCD() {                        // วาดข้อความลงจอจริง
  for (int r = 0; r < 4; r++) {         // วนทีละบรรทัด
    lcd.setCursor(0, r);                // ย้ายเคอร์เซอร์ไปต้นบรรทัด r
    lcd.print(lcdLines[r]);             // พิมพ์ข้อความบรรทัดนั้น
  }
}

// ===== ไม้กั้น =====
void setEntryGate(int angle) { entryAngle = angle; entryServo.write(angle); } // สั่งไม้กั้นเข้า พร้อมจำมุม
void setExitGate(int angle)  { exitAngle = angle;  exitServo.write(angle); Serial.printf("EXIT servo -> %d (GPIO %d)\n", angle, SERVO_OUT_PIN); } // สั่งไม้กั้นออก จำมุม และพิมพ์ log

// ===== WiFi / MQTT =====
void connectWiFi() {                    // เชื่อมต่อ WiFi (เรียกซ้ำได้ทุกรอบ)
  if (WiFi.status() == WL_CONNECTED) return; // ต่ออยู่แล้วไม่ต้องทำอะไร
  if (lastWifiTry != 0 && millis() - lastWifiTry < 10000) return; // ลองใหม่ได้ทุก 10 วินาที
  lastWifiTry = millis();               // จดเวลาที่ลองครั้งนี้
  WiFi.disconnect();                    // ตัดการเชื่อมต่อเก่า
  WiFi.begin(WIFI_SSID, WIFI_PASS);     // เริ่มเชื่อมต่อใหม่
}

void connectMQTT() {                    // เชื่อมต่อ MQTT (เรียกซ้ำได้ทุกรอบ)
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return; // WiFi ยังไม่ติด หรือ MQTT ต่ออยู่แล้ว -> ข้าม
  if (lastMqttTry != 0 && millis() - lastMqttTry < 5000) return; // ลองใหม่ได้ทุก 5 วินาที
  lastMqttTry = millis();               // จดเวลาที่ลอง
  String cid = "esp32-parking-" + String((uint32_t)ESP.getEfuseMac(), HEX); // สร้าง client ID จาก MAC
  if (mqtt.connect(cid.c_str())) {      // ลองเชื่อมต่อ
    Serial.println("MQTT connected");   // สำเร็จ แจ้งใน Serial
    lastHeartbeat = 0;                  // บังคับส่งข้อมูลทันที
  } else {                              // ไม่สำเร็จ
    Serial.print("MQTT failed, rc=");   // แจ้งว่าล้มเหลว
    Serial.println(mqtt.state());       // พิมพ์รหัสข้อผิดพลาด
  }
}

void publishIfChanged(const char* topic, const String& payload, String& last, bool force) { // ส่ง payload เมื่อเปลี่ยนหรือบังคับ
  if (!mqtt.connected()) return;        // ไม่ได้ต่อ MQTT ไม่ต้องส่ง
  if (force || payload != last) {       // บังคับส่ง หรือข้อมูลต่างจากครั้งก่อน
    if (mqtt.publish(topic, payload.c_str())) last = payload; // ส่งสำเร็จค่อยจำไว้
  }
}

void publishAll(const String& slots, bool in1, bool in2, bool ex, const String& entryAlert) { // ส่งข้อมูลทุก topic ขึ้นเว็บ
  bool force = false;                   // ค่าเริ่มต้น ไม่บังคับส่ง
  if (millis() - lastHeartbeat >= HEARTBEAT_MS) { // ถึงเวลาส่ง heartbeat
    force = true;                       // บังคับส่งรอบนี้
    lastHeartbeat = millis();           // จดเวลา
  }

  String zoneCore = "\"s\":\"" + slots + "\""; // ส่วนหลักของ payload ช่องจอด เช่น "s":"1010"
  if (straddle) zoneCore += ",\"alert\":\"STRADDLE\",\"p\":\"" + problemMask + "\""; // ถ้าจอดคร่อม เพิ่มธงและ mask
  String tArr = "[";                    // เริ่มอาเรย์เวลาจอดของแต่ละช่อง
  uint32_t nowSec = nowEpoch();         // เวลาปัจจุบัน
  for (int i = 0; i < 4; i++) {         // วนทุกช่อง
    uint32_t sec = (slots[i] == '1' && slotSince[i] != 0 && nowSec > slotSince[i]) ? nowSec - slotSince[i] : 0; // วินาทีที่จอดมา (ช่องว่าง = 0)
    tArr += String(sec);                // ต่อค่าเข้าอาเรย์
    if (i < 3) tArr += ",";             // ใส่คอมมาคั่น ยกเว้นตัวสุดท้าย
  }
  tArr += "]";                          // ปิดอาเรย์
  String zoneFull = "{" + zoneCore + ",\"t\":" + tArr + "}"; // ประกอบ JSON ช่องจอดเต็ม
  if (mqtt.connected() && (force || zoneCore != lastZone)) { // ต่ออยู่ และ (บังคับ หรือ ช่องเปลี่ยน)
    if (mqtt.publish(TOPIC_ZONE, zoneFull.c_str())) lastZone = zoneCore; // ส่งแล้วจำค่า
  }

  int pending = (in1 && entState == ENT_IDLE) ? 1 : 0; // มีรถรอเข้า: S1 เจอรถและไม้กั้นยังไม่เปิด
  String inPayload = "{\"sensor\":" + String(in1 ? 1 : 0) + // สถานะ S1
                     ",\"sensor2\":" + String(in2 ? 1 : 0) + // สถานะ S2
                     ",\"servo\":" + String(entryAngle) +   // มุมไม้กั้นเข้า
                     ",\"pass\":" + String(passIn) +        // จำนวนรถเข้าสะสม
                     ",\"pending\":" + String(pending) +    // จำนวนรถรอเข้า
                     ",\"alert\":\"" + entryAlert + "\"}";  // ข้อความแจ้งเตือนทางเข้า
  publishIfChanged(TOPIC_IN, inPayload, lastIn, force); // ส่งข้อมูลทางเข้า

  String outPayload = "{\"sensor\":" + String(ex ? 1 : 0) + // สถานะเซนเซอร์ทางออก
                      ",\"servo\":" + String(exitAngle) +   // มุมไม้กั้นออก
                      ",\"pass\":" + String(passOut) +      // จำนวนรถออกสะสม
                      ",\"alert\":\"\"}";                   // ทางออกไม่มีแจ้งเตือน
  publishIfChanged(TOPIC_OUT, outPayload, lastOut, force); // ส่งข้อมูลทางออก

  String lcdPayload = "{\"lines\":[";   // เริ่ม JSON ของจอ LCD
  for (int i = 0; i < 4; i++) {         // วนทั้ง 4 บรรทัด
    lcdPayload += "\"" + lcdLines[i] + "\""; // ใส่ข้อความแต่ละบรรทัดในเครื่องหมายคำพูด
    if (i < 3) lcdPayload += ",";       // คั่นด้วยคอมมา
  }
  lcdPayload += "]}";                   // ปิด JSON
  publishIfChanged(TOPIC_LCD, lcdPayload, lastLcd, force); // ส่งข้อมูลจอ LCD
}

// ===== S2 Interrupt =====
volatile unsigned long s2LowSince = 0;  // เวลาที่ S2 เริ่มเจอรถ (ตั้งโดย interrupt)
volatile unsigned long s2LastRise = 0;  // เวลาที่ S2 หายไปล่าสุด
volatile unsigned long s2StartKeep = 0; // เวลาเริ่มของช่วงที่เพิ่งจบ (ไว้ต่อกับช่วงถัดไปถ้าห่างสั้น)
volatile unsigned long s2BestMs = 0;    // ช่วงเจอรถที่นานที่สุดที่ interrupt จับได้
void IRAM_ATTR s2ISR() {                // ฟังก์ชัน interrupt ทำงานทุกครั้งที่ S2 เปลี่ยนสถานะ
  unsigned long now = millis();         // เวลาปัจจุบัน
  if (digitalRead(IR_IN_2) == DETECTED) { // S2 เจอวัตถุ
    if (s2LowSince == 0) s2LowSince = (s2StartKeep != 0 && now - s2LastRise <= S2_GAP_MS) ? s2StartKeep : now; // เริ่มช่วงใหม่ หรือต่อจากช่วงเดิมถ้าห่างไม่เกิน 60 ms
  } else if (s2LowSince != 0) {         // S2 ว่างลง และเคยเจอวัตถุอยู่
    s2StartKeep = s2LowSince; s2LastRise = now; // จำจุดเริ่มและเวลาที่หาย
    unsigned long d = now - s2LowSince; // ความยาวของช่วงที่เจอวัตถุ
    if (d > s2BestMs) s2BestMs = d;     // เก็บค่าที่นานที่สุด
    s2LowSince = 0;                     // จบช่วง
  }
}

void setup() {                          // ทำงานครั้งเดียวตอนเปิดเครื่อง
  Serial.begin(115200);                 // เปิด Serial สำหรับดีบัก

  pinMode(IR_IN_1, INPUT);              // S1 เป็นอินพุต
  pinMode(IR_IN_2, INPUT);              // S2 เป็นอินพุต
  attachInterrupt(digitalPinToInterrupt(IR_IN_2), s2ISR, CHANGE); // ผูก interrupt กับ S2 ทุกการเปลี่ยนระดับ
  pinMode(IR_EXIT, INPUT_PULLUP);       // เซนเซอร์ทางออกเป็นอินพุตพร้อมตัวต้านทาน pull-up
  for (int i = 0; i < 4; i++) {         // วนทุกช่องจอด
    pinMode(slotPins[i], INPUT);        // ตั้งขาเซนเซอร์ช่องจอดเป็นอินพุต
  }

  Wire.begin(SDA_PIN, SCL_PIN);         // เริ่ม I2C
  lcd.init();                           // เริ่มจอ LCD
  lcd.backlight();                      // เปิดไฟหลังจอ

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); // จองไทเมอร์ PWM ให้เซอร์โว 2 ตัว
  entryServo.setPeriodHertz(50); exitServo.setPeriodHertz(50); // ตั้งความถี่เซอร์โว 50 Hz
  entryServo.attach(SERVO_IN_PIN, 500, 2400); // ผูกเซอร์โวทางเข้ากับขา (พัลส์ 500-2400 µs)
  exitServo.attach(SERVO_OUT_PIN, 500, 2400); // ผูกเซอร์โวทางออกกับขา
  setEntryGate(GATE_DOWN);              // เริ่มต้นปิดไม้กั้นเข้า
  setExitGate(GATE_DOWN);               // เริ่มต้นปิดไม้กั้นออก

  snprintf(TOPIC_ZONE, sizeof(TOPIC_ZONE), "%s/zone/%s", BASE, ZONE); // ประกอบ topic ช่องจอด
  snprintf(TOPIC_IN,   sizeof(TOPIC_IN),   "%s/gate/in",  BASE);      // ประกอบ topic ทางเข้า
  snprintf(TOPIC_OUT,  sizeof(TOPIC_OUT),  "%s/gate/out", BASE);      // ประกอบ topic ทางออก
  snprintf(TOPIC_LCD,  sizeof(TOPIC_LCD),  "%s/lcd",      BASE);      // ประกอบ topic จอ LCD

  prefs.begin("parking", false);        // เปิดพื้นที่เก็บค่าชื่อ "parking" (อ่าน/เขียนได้)
  for (int i = 0; i < 4; i++) {         // วนทุกช่อง
    char key[4];                        // ที่เก็บชื่อคีย์
    snprintf(key, sizeof(key), "s%d", i); // สร้างชื่อคีย์
    slotSince[i] = prefs.getUInt(key, 0); // อ่านเวลาจอดที่บันทึกไว้
    if (CLEAR_TIMERS_ON_BOOT && slotSince[i] != 0) { slotSince[i] = 0; saveSlot(i); } // ล้างค่าเก่าตอนบูต
  }
  configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com"); // ตั้งเวลา NTP โซนไทย (UTC+7)

  WiFi.mode(WIFI_STA);                  // ใช้ WiFi โหมดสถานี (ต่อกับเราเตอร์)
  mqtt.setServer(MQTT_HOST, MQTT_PORT); // ตั้งที่อยู่ broker
  mqtt.setBufferSize(512);              // ขยายบัฟเฟอร์ข้อความ MQTT เป็น 512 ไบต์
  mqtt.setSocketTimeout(2);             // timeout ของซ็อกเก็ต 2 วินาที
  connectWiFi();                        // เริ่มเชื่อมต่อ WiFi
  Serial.println("setup done");         // แจ้งว่า setup เสร็จ
}

void loop() {                           // ทำงานวนซ้ำตลอด
  unsigned long currentMillis = millis(); // เวลาปัจจุบัน (ms) ของรอบนี้

  connectWiFi();                        // ดูแลการเชื่อมต่อ WiFi
  connectMQTT();                        // ดูแลการเชื่อมต่อ MQTT
  mqtt.loop();                          // ให้ไลบรารี MQTT ประมวลผลงานค้าง

  // ---------- 1. อ่านช่องจอด ----------
  String currentSlots = "";             // สถานะ 4 ช่อง เช่น "1010"
  int freeSlots = 0;                    // จำนวนช่องว่าง
  for (int i = 0; i < 4; i++) {         // วนทีละช่อง
    bool raw = (digitalRead(slotPins[i]) == DETECTED); // ค่าดิบ: เจอรถหรือไม่
    if (!slotInit) { slotRaw[i] = raw; slotStable[i] = raw; slotRawSince[i] = currentMillis; } // รอบแรก ตั้งค่าเริ่มต้นตามเซนเซอร์
    if (raw != slotRaw[i]) { slotRaw[i] = raw; slotRawSince[i] = currentMillis; } // ค่าดิบเปลี่ยน -> จดเวลา
    if (slotRaw[i] != slotStable[i]) {  // ค่าดิบต่างจากค่าที่กรองแล้ว
      unsigned long need = slotRaw[i] ? SLOT_OCC_CONFIRM_MS : SLOT_FREE_CONFIRM_MS; // เวลาที่ต้องรอ (รถเข้า 0.3 วิ / รถออก 2 วิ)
      if (currentMillis - slotRawSince[i] >= need) slotStable[i] = slotRaw[i]; // เปลี่ยนนานพอ -> ยอมรับค่าใหม่
    }
    bool isOccupied = slotStable[i];    // ใช้ค่าที่กรองแล้ว
    currentSlots += isOccupied ? "1" : "0"; // ต่อเข้าสตริงสถานะ
    if (!isOccupied) freeSlots++;       // นับช่องว่าง
    if (isOccupied && (prevSlots.length() != 4 || prevSlots[i] == '0')) occAt[i] = currentMillis; // เพิ่งมีรถเข้าช่องนี้ จดเวลา
  }
  slotInit = true;                      // อ่านรอบแรกเสร็จแล้ว
  int occupiedCount = TOTAL_SLOTS - freeSlots; // [แก้ไข] จำนวนช่องที่มีรถ
  if (!countSynced) {                   // ถ้ายังไม่ได้ซิงค์ตัวนับตอนเปิดเครื่อง
    passCount = occupiedCount;          // ตั้งตัวนับเท่ากับช่องที่มีรถ
    passIn = occupiedCount;             // ตั้งยอดเข้าสะสมเท่ากัน
    countSynced = true;                 // ทำเครื่องหมายว่าซิงค์แล้ว
  }

  int excess = occupiedCount - passCount; // ช่องที่มีรถ เกิน ตัวนับกี่ช่อง
  if (excess > 0) {                     // ถ้าเกิน (เช่น รถจอดคร่อมกินสองช่อง)
    if (mismatchSince == 0) mismatchSince = currentMillis; // เริ่มจับเวลา
    if (currentMillis - mismatchSince > MISMATCH_CONFIRM_MS) straddle = true; // เกิน 5 วินาที -> ยืนยันว่าจอดคร่อม
  } else {                              // ไม่เกิน
    mismatchSince = 0;                  // ล้างตัวจับเวลา
    straddle = false;                   // ไม่มีการจอดคร่อม
  }

  // [แก้ไข] ซิงค์ passCount กลับให้ตรงช่องจริง ถ้านับเกินค้างนาน และไม่มีรถเคลื่อนไหวที่ประตู
  if (passCount > occupiedCount && entState == ENT_IDLE && exitState == EX_IDLE) { // [แก้ไข] ตัวนับมากกว่าช่องจริง และประตูทั้งสองว่าง
    if (overCountSince == 0) overCountSince = currentMillis; // [แก้ไข] เริ่มจับเวลา
    if (currentMillis - overCountSince > COUNT_RESYNC_MS) { // [แก้ไข] ผิดปกติเกิน 60 วินาที
      Serial.printf("RESYNC passCount %d -> %d\n", passCount, occupiedCount); // [แก้ไข] แจ้งใน Serial ว่ารีเซ็ต
      passCount = occupiedCount;        // [แก้ไข] ตั้งตัวนับให้เท่าช่องที่มีรถจริง
      passIn = passOut + (unsigned long)occupiedCount; // [แก้ไข] ปรับยอดเข้าให้ "รถภายใน" บนเว็บตรงกัน
      overCountSince = 0;               // [แก้ไข] ล้างตัวจับเวลา
    }
  } else {                              // เงื่อนไขไม่เข้า
    overCountSince = 0;                 // [แก้ไข] ล้างตัวจับเวลา
  }

  String mask = "0000";                 // mask ช่องที่มีปัญหา เริ่มต้นไม่มี
  if (straddle) {                       // ถ้าจอดคร่อม
    int need = min(excess * 2, occupiedCount); // จำนวนช่องที่ต้องทำเครื่องหมาย (คันละ 2 ช่อง)
    bool used[4] = {false, false, false, false}; // ช่องที่เลือกไปแล้ว
    for (int k = 0; k < need; k++) {    // เลือกให้ครบตามจำนวน
      int best = -1;                    // ช่องที่ดีที่สุดรอบนี้
      for (int i = 0; i < 4; i++) {     // วนทุกช่อง
        if (currentSlots[i] == '1' && !used[i] && (best < 0 || (long)(occAt[i] - occAt[best]) > 0)) best = i; // เลือกช่องที่มีรถและเพิ่งมาล่าสุด
      }
      if (best >= 0) { used[best] = true; mask.setCharAt(best, '1'); } // ทำเครื่องหมายช่องนั้นว่ามีปัญหา
    }
  }
  problemMask = mask;                   // เก็บ mask ไว้ส่งขึ้นเว็บ
  updateSlotTimers(currentSlots);       // อัปเดตตัวจับเวลาจอดของแต่ละช่อง

  // ---------- 2. อ่านเซนเซอร์ทางเข้า + ตรวจขับจี้ท้าย ----------
  bool inSensor1 = (digitalRead(IR_IN_1) == DETECTED); // S1 เจอรถหรือไม่
  bool inSensor2 = (digitalRead(IR_IN_2) == DETECTED); // S2 เจอรถหรือไม่

  if (inSensor1) {                      // S1 เจอรถ
    lastS1At = currentMillis;           // จดเวลาล่าสุดที่เจอ
    if (s1DetectSince == 0) s1DetectSince = currentMillis; // เริ่มนับเวลาที่เจอต่อเนื่อง
  } else {                              // S1 ว่าง
    s1DetectSince = 0;                  // ล้างตัวนับต่อเนื่อง
  }
  if (!inSensor1) s1Lock = false;       // S1 ว่างแล้ว ปลดล็อกไม้กั้น
  bool s1Held = (s1DetectSince != 0 && currentMillis - s1DetectSince >= S1_HOLD_MS) && !s1Lock; // S1 เจอรถนานพอ และไม่ถูกล็อก

  bool tailgate = false;                // ธงขับจี้ท้าย
  if (inSensor1 && inSensor2) {         // S1 และ S2 เจอรถพร้อมกัน
    if (tailGateTimer == 0) tailGateTimer = currentMillis; // เริ่มจับเวลา
    if (currentMillis - tailGateTimer > 3000) tailgate = true; // เกิน 3 วินาที = ขับจี้ท้าย
  } else {                              // ไม่ได้เจอพร้อมกัน
    tailGateTimer = 0;                  // ล้างตัวจับเวลา
  }

  // ---------- 2.5 นับรถเข้าด้วย S2 ตัวเดียว ----------
  unsigned long s2Low, s2Best;          // สำเนาค่าจาก interrupt
  noInterrupts(); s2Low = s2LowSince; s2Best = s2BestMs; s2BestMs = 0; interrupts(); // อ่านค่าแบบปลอดภัย แล้วล้าง s2BestMs
  // [แก้ไข] ช่วงหน่วง: หลังยกไม้กั้นเข้า ยังไม่ฟัง S2 จนกว่าจะครบ ENTRY_S2_DELAY_MS (กันคนเดินผ่านแล้วไม้กั้นปิดทับรถ)
  bool s2Ignore = (entState == ENT_WAIT_PASS) && (currentMillis - entTimer < ENTRY_S2_DELAY_MS); // [แก้ไข] true = ไม้กั้นเพิ่งยก ยังอยู่ในช่วงหน่วง ให้เมิน S2
  bool s2Pulse = !s2Ignore && (inSensor2 || s2Best > 0); // [แก้ไข] S2 มีสัญญาณ และไม่อยู่ในช่วงหน่วง
  bool s2CarLike = !s2Ignore && ((s2Best >= CAR_MIN_MS) || (s2Low != 0 && currentMillis - s2Low >= CAR_MIN_MS)); // [แก้ไข] สัญญาณนานพอเป็นรถ และไม่อยู่ในช่วงหน่วง (คนเดินผ่านจึงไม่ถูกนับ)
  if (s2Armed) {                        // พร้อมนับรถคันใหม่
    if (s2CarLike && lastS1At != 0 && currentMillis - lastS1At <= S1_TO_S2_MS) { // เหมือนรถ และเพิ่งผ่าน S1 มา
      passCount++;                      // เพิ่มจำนวนรถในลาน
      passIn++;                         // เพิ่มยอดเข้าสะสม
      s2Armed = false;                  // ไม่นับซ้ำจนกว่าจะว่าง
      s2ClearSince = 0;                 // ล้างตัวจับเวลาว่าง
    }
  } else {                              // ยังไม่พร้อมนับ (รอ S2 ว่าง)
    if (!inSensor2 && s2Low == 0) {     // S2 ว่างจริง
      if (s2ClearSince == 0) s2ClearSince = currentMillis; // เริ่มจับเวลาว่าง
      if (currentMillis - s2ClearSince >= S2_REARM_MS) s2Armed = true; // ว่างนานพอ พร้อมนับคันต่อไป
    } else {                            // S2 ยังไม่ว่าง
      s2ClearSince = 0;                 // เริ่มนับใหม่
    }
  }

  // ---------- 3. เลือกข้อความแจ้งเตือน ----------
  // [แก้ไข] ลานเต็ม = เซนเซอร์ช่องเต็ม "หรือ" ตัวนับรถในลานถึง 4 คัน
  bool lotFull = (freeSlots == 0) || (passCount >= TOTAL_SLOTS); // [แก้ไข] รถที่กำลังออกยังนับเป็นรถในลานจนพ้นไม้กั้น
  if (tailgate)              alertMsg = " WARNING: TAILGATE! "; // ขับจี้ท้าย (ลำดับความสำคัญสูงสุด)
  else if (straddle)         alertMsg = " WARNING: STRADDLE! "; // จอดคร่อมช่อง
  else if (lotFull)          alertMsg = "   ** LOT FULL **   "; // ลานเต็ม
  else                       alertMsg = welcomeMsg;              // ปกติ แสดงข้อความต้อนรับ

  // ---------- 4. อัปเดตจอ LCD ----------
  if (currentMillis - lastBlink >= BLINK_MS) { // ถึงเวลาสลับบรรทัดแรก
    lastBlink = currentMillis;          // จดเวลา
    blinkStatus = !blinkStatus;         // สลับสถานะ
  }
  String sig = currentSlots + alertMsg + (blinkStatus ? "S" : "T"); // ลายเซ็นของสิ่งที่ควรแสดง
  bool forceDraw = (currentMillis - lastDiag > 5000); // ครบ 5 วินาที บังคับวาดใหม่ (กันจอเพี้ยน)
  if (sig != shownSig || forceDraw) {   // ข้อมูลเปลี่ยน หรือบังคับวาด
    buildLcdLines(freeSlots, currentSlots); // สร้างข้อความ 4 บรรทัด
    drawLCD();                          // วาดลงจอ
    shownSig = sig;                     // จำสิ่งที่แสดงอยู่
  }
  if (forceDraw) {                      // ทุก 5 วินาที
    lastDiag = currentMillis;           // จดเวลา
    Serial.printf("alive | wifi=%d mqtt=%d | slots=%s | inside=%d in=%lu out=%lu | lotFull=%d exit=%d\n", // [แก้ไข] พิมพ์สถานะรวม + lotFull + exitState
                  WiFi.status() == WL_CONNECTED, mqtt.connected(), // สถานะ WiFi และ MQTT
                  currentSlots.c_str(), passCount, passIn, passOut, lotFull, (int)exitState); // ช่องจอด จำนวนรถ ยอดเข้า/ออก ลานเต็มไหม สถานะทางออก
  }

  // ---------- 5. ไม้กั้นทางเข้า ----------
  // [แก้ไข] ไม่เปิดถ้ายังเต็ม และไม่เปิดขณะที่ทางออกกำลังทำงาน (รถออกมีสิทธิ์ก่อน)
  switch (entState) {                   // แยกตามสถานะทางเข้า
    case ENT_IDLE:                      // สถานะว่าง รอรถ
      if (s1Held && !lotFull && exitState == EX_IDLE) { // [แก้ไข] รถรอที่ S1 นานพอ + ลานไม่เต็ม + ทางออกว่าง
        setEntryGate(GATE_UP);          // ยกไม้กั้นเข้า
        entState = ENT_WAIT_PASS;       // เปลี่ยนเป็นรอรถผ่าน
        entTimer = currentMillis;       // จดเวลาที่เปิด
      }
      break;                            // จบ case

    case ENT_WAIT_PASS:                 // ไม้กั้นเปิด รอรถผ่าน
      if (s2Pulse) {                    // S2 เจอรถ (รถเริ่มผ่านเข้ามา)
        setEntryGate(GATE_DOWN);        // ปิดไม้กั้น
        entState = ENT_CLEARING;        // เปลี่ยนเป็นรอเคลียร์
      }
      else if (currentMillis - entTimer > 10000) { // เปิดนาน 10 วินาทีแล้วไม่มีรถผ่าน
        setEntryGate(GATE_DOWN);        // ปิดไม้กั้น
        entState = ENT_IDLE;            // กลับสถานะว่าง
        s1Lock = true;                  // ล็อกไม่ให้เปิดซ้ำจนกว่า S1 จะว่าง
        timeoutAlertUntil = currentMillis + 5000; // แสดงแจ้งเตือน timeout 5 วินาที
      }
      break;                            // จบ case

    case ENT_CLEARING:                  // รอรถผ่านพ้นเซนเซอร์ทั้งสอง
      if (!inSensor2 && !inSensor1) {   // S1 และ S2 ว่างหมด
        entState = ENT_IDLE;            // กลับสถานะว่าง
      }
      break;                            // จบ case
  }

  // ---------- 6. ไม้กั้นทางออก (ทำงานอิสระ ไม่สนว่าที่จอดจะเต็มหรือไม่) ----------
  bool exitSensor = (digitalRead(IR_EXIT) == DETECTED); // เซนเซอร์ทางออกเจอรถหรือไม่
  if (exitSensor) {                     // เจอรถ
    if (exitDetectSince == 0) exitDetectSince = currentMillis; // เริ่มนับเวลาที่เจอต่อเนื่อง
  } else {                              // ไม่เจอ
    exitDetectSince = 0;                // ล้างตัวนับ
  }

  bool exitHeld = exitSensor && (currentMillis - exitDetectSince >= EXIT_HOLD_MS); // เจอรถนานพอ (EXIT_HOLD_MS = 0 คือทันที)

  switch (exitState) {                  // แยกตามสถานะทางออก
    case EX_IDLE:                       // สถานะว่าง รอรถ
      if (exitHeld) {                   // เจอรถที่ทางออก
        setExitGate(GATE_UP);           // ยกไม้กั้นออกทันที
        exitState = EX_OPEN;            // เปลี่ยนเป็นสถานะเปิด
        exitTimer = currentMillis;      // จดเวลาล่าสุดที่เจอรถ
      }
      break;                            // จบ case

    case EX_OPEN:                       // ไม้กั้นออกเปิดอยู่
      {                                 // เปิดบล็อกเพื่อประกาศตัวแปร static
        static unsigned long lastReassert = 0; // เวลาที่สั่งเซอร์โวซ้ำล่าสุด
        if (currentMillis - lastReassert > 300) { // ทุก 300 ms
          exitServo.write(exitAngle);   // สั่งเซอร์โวซ้ำ ให้ค้างตำแหน่งเปิดแน่นอน
          lastReassert = currentMillis; // จดเวลา
        }
      }
      if (exitSensor) exitTimer = currentMillis; // ถ้ารถยังอยู่ที่เซนเซอร์ รีเซ็ตเวลารอปิด

      if (currentMillis - exitTimer > 3000) { // รถพ้นเซนเซอร์นาน 3 วินาที
        setExitGate(GATE_DOWN);         // ปิดไม้กั้นออก
        if (passCount > 0) passCount--; // รถออกพ้นจริง ลดจำนวนรถในลาน (ปล่อยที่ว่างให้รถที่รอเข้า)
        passOut++;                      // เพิ่มยอดออกสะสม
        exitState = EX_IDLE;            // กลับสถานะว่าง
      }
      break;                            // จบ case
  }

  // ---------- 7. ส่งข้อมูลขึ้นเว็บ ----------
  String entryAlert = "";               // ข้อความแจ้งเตือนทางเข้า
  if (tailgate) entryAlert = "TAILGATE"; // ขับจี้ท้าย
  else if ((long)(timeoutAlertUntil - currentMillis) > 0) entryAlert = "ENTRY TIMEOUT"; // ยังอยู่ในช่วงแสดง timeout
  else if (s1Lock && inSensor1) entryAlert = "OBSTRUCTION"; // ถูกล็อกและ S1 ยังเจอวัตถุ (มีของขวาง)
  else if (inSensor1 && entState == ENT_IDLE && lotFull) entryAlert = "LOT FULL"; // มีรถรอแต่ลานเต็ม

  publishAll(currentSlots, inSensor1, inSensor2, exitSensor, entryAlert); // ส่งข้อมูลทั้งหมดขึ้น MQTT

  prevSlots = currentSlots;             // จำสถานะช่องรอบนี้ไว้เทียบรอบหน้า
  delay(20);                            // หน่วง 20 ms ลดภาระ CPU
}
