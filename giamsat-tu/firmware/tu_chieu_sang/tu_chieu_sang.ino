/*
 * ============================================================================
 *  GATEWAY TỦ CHIẾU SÁNG CÔNG CỘNG – ESP32
 * ============================================================================
 *  Chức năng:
 *   - Đọc điện áp / dòng / công suất / điện năng 3 pha qua Modbus RTU (RS485)
 *     (mặc định: 3 đồng hồ PZEM-016, địa chỉ 1-2-3, mỗi pha 1 cái)
 *   - Cảm biến rung / nghiêng MPU6050  -> phát hiện xe va chạm, trụ/tủ bị nghiêng
 *   - Công tắc cửa tủ (reed switch)    -> phát hiện mở tủ trái phép
 *   - Relay điều khiển cuộn hút contactor theo lịch bật/tắt (NTP, giờ VN)
 *   - Gửi dữ liệu + cảnh báo lên MQTT, nhận lệnh bật/tắt từ xa
 *   - Ghi log cảnh báo ra thẻ SD, giữ hàng đợi khi mất mạng rồi gửi bù
 *
 *  Cảnh báo (code):
 *   CAT_CAP     Pha đang có tải bỗng mất dòng, điện áp vẫn còn -> nghi cắt/trộm cáp
 *   MAT_PHA     Mất điện áp 1-2 pha
 *   MAT_DIEN    Mất điện lưới cả 3 pha
 *   DIEN_AP     Quá áp / sụt áp kéo dài
 *   KHONG_SANG  Đã đến giờ bật, relay đóng nhưng không có dòng -> hỏng contactor/CB/timer
 *   SANG_NGAY   Relay đang cắt nhưng vẫn có dòng -> dính tiếp điểm contactor
 *   VA_CHAM     Rung động mạnh đột ngột (xe tông)
 *   NGHIENG     Tủ/trụ bị nghiêng so với lúc lắp đặt
 *   MO_CUA      Cửa tủ mở ngoài chế độ bảo trì
 *   LOI_DONG_HO Không đọc được đồng hồ điện qua RS485
 *
 *  SIM_MODE = 1: chạy trên Wokwi (wokwi.com) không cần phần cứng:
 *   3 biến trở = dòng điện pha A/B/C (0-10A), công tắc gạt = điện lưới,
 *   nút gạt cửa = cửa tủ, LED = relay contactor, MPU6050 = cảm biến rung.
 *
 *  Thư viện: PubSubClient (Nick O'Leary), ModbusMaster (Doc Walker – chỉ khi SIM_MODE 0)
 * ============================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <math.h>

// ============================== CẤU HÌNH ==================================
#define SIM_MODE 1                         // 1 = giả lập Wokwi, 0 = phần cứng thật

const char* WIFI_SSID   = "Wokwi-GUEST";   // Wi-Fi hoặc router 4G trong tủ
const char* WIFI_PASS   = "";
const char* MQTT_HOST   = "broker.hivemq.com";  // broker công cộng – CHỈ để thử nghiệm
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER   = "";
const char* MQTT_PASS   = "";
// Đổi TOPIC_ROOT thành chuỗi riêng, khó đoán (broker công cộng ai cũng nghe được).
// Phải trùng với ô "Mã kênh" trong trang mô phỏng.
const char* TOPIC_ROOT  = "cs/demo-doi-cs";
const char* CABINET_ID  = "TU-001";

// Lịch bật/tắt đèn (giờ Việt Nam)
const int ON_HOUR = 18, ON_MIN = 0;
const int OFF_HOUR = 5, OFF_MIN = 30;

// Ngưỡng phát hiện
const float I_LOADED      = 1.0;   // A  – pha được coi là đang có tải
const float I_ZERO        = 0.2;   // A  – coi như mất dòng
const float V_LOST        = 100;   // V  – dưới mức này là mất pha
const float V_MIN_OK      = 180;   // V  – sụt áp
const float V_MAX_OK      = 250;   // V  – quá áp
const float TILT_ALARM    = 15.0;  // độ
const float VIB_ALARM_G   = 0.6;   // g  – thay đổi gia tốc đột ngột
const uint8_t DEBOUNCE_N  = 3;     // số lần đọc liên tiếp mới báo (lọc nhiễu)
const uint8_t VOLT_DEBOUNCE_N = 10;
const uint32_t RELAY_GRACE_MS   = 30000;   // chờ đèn khởi động sau khi đóng relay
const uint32_t READ_MS          = 1000;
const uint32_t TELEMETRY_MS     = 10000;
const uint32_t SD_TELE_MS       = 60000;
const uint32_t COLLISION_COOLDOWN_MS = 60000;

// Chân kết nối
const int PIN_RELAY    = 26;   // relay -> cuộn hút contactor (SIM: LED)
const int PIN_DOOR     = 27;   // reed switch cửa tủ, INPUT_PULLUP, HIGH = đang mở
const int PIN_SD_CS    = 5;    // thẻ SD (SPI: SCK18 MISO19 MOSI23)
const int PIN_SDA      = 21, PIN_SCL = 22;     // MPU6050
const int PIN_RS485_RX = 16, PIN_RS485_TX = 17, PIN_RS485_DE = 4;
#if SIM_MODE
const int PIN_POT[3]   = {34, 35, 32};          // biến trở thay dòng pha A/B/C
const int PIN_MAINS    = 33;                    // công tắc gạt: HIGH = có điện lưới
#endif
// ==========================================================================

#if !SIM_MODE
#include <ModbusMaster.h>
ModbusMaster meter[3];
void rs485Tx() { digitalWrite(PIN_RS485_DE, HIGH); }
void rs485Rx() { digitalWrite(PIN_RS485_DE, LOW); }
#endif

WiFiClient   net;
PubSubClient mqtt(net);

const char PH_NAME[3] = {'A', 'B', 'C'};

struct PhaseData { float v, i, p; double wh; bool ok; };
PhaseData ph[3];

// ---- Trạng thái điều khiển ----
enum Mode { MODE_AUTO, MODE_ON, MODE_OFF };
Mode mode = SIM_MODE ? MODE_ON : MODE_AUTO;   // SIM: bật sẵn để thử bất kỳ giờ nào
bool relayOn = false;
uint32_t relayChangedAt = 0;
bool maintenance = false;          // chế độ bảo trì: không báo mở cửa
bool phaseWasLoaded[3] = {false, false, false};
bool anyLoadSinceOn = false;

// ---- MPU6050 ----
bool  mpuOk = false;
float refAx = 0, refAy = 0, refAz = 1;
float lastAx = 0, lastAy = 0, lastAz = 1;
float vibPeak = 0;                 // rung lớn nhất trong chu kỳ đọc
float tiltDeg = 0;
uint32_t lastCollisionAt = 0;
bool  collisionEver = false;

// ---- Đồng hồ điện ----
uint8_t meterFail[3] = {0, 0, 0};

// ---- SD + hàng đợi ----
bool sdOk = false;
const int QUEUE_MAX = 20;
String queueTopic[QUEUE_MAX];
String queueMsg[QUEUE_MAX];
int queueLen = 0;

// ---- Bảng cảnh báo ----
enum AlarmId {
  AL_CAT_CAP_A, AL_CAT_CAP_B, AL_CAT_CAP_C,
  AL_MAT_PHA_A, AL_MAT_PHA_B, AL_MAT_PHA_C,
  AL_MAT_DIEN, AL_DIEN_AP, AL_KHONG_SANG, AL_SANG_NGAY,
  AL_NGHIENG, AL_MO_CUA, AL_LOI_DONG_HO,
  AL_COUNT
};
struct Alarm { const char* code; char phase; uint8_t cnt; bool active; };
Alarm alarms[AL_COUNT] = {
  {"CAT_CAP", 'A', 0, false}, {"CAT_CAP", 'B', 0, false}, {"CAT_CAP", 'C', 0, false},
  {"MAT_PHA", 'A', 0, false}, {"MAT_PHA", 'B', 0, false}, {"MAT_PHA", 'C', 0, false},
  {"MAT_DIEN", '-', 0, false}, {"DIEN_AP", '-', 0, false},
  {"KHONG_SANG", '-', 0, false}, {"SANG_NGAY", '-', 0, false},
  {"NGHIENG", '-', 0, false}, {"MO_CUA", '-', 0, false}, {"LOI_DONG_HO", '-', 0, false},
};

uint32_t lastRead = 0, lastTele = 0, lastSdTele = 0, lastMqttTry = 0, bootMs = 0;

// ==========================================================================
//  TIỆN ÍCH
// ==========================================================================
String topic(const char* leaf) {
  return String(TOPIC_ROOT) + "/tu/" + CABINET_ID + "/" + leaf;
}

bool timeValid() {
  return time(nullptr) > 1700000000;   // đã đồng bộ NTP
}

long long nowEpoch() {
  return timeValid() ? (long long)time(nullptr) : 0;
}

void sdAppend(const char* path, const String& line) {
  if (!sdOk) return;
  File f = SD.open(path, FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

void publishOrQueue(const String& t, const String& msg, bool retain = false) {
  if (mqtt.connected() && mqtt.publish(t.c_str(), msg.c_str(), retain)) return;
  if (retain) return;                       // bản tin trạng thái: không cần gửi bù
  if (queueLen >= QUEUE_MAX) {              // đầy: bỏ bản tin cũ nhất
    for (int k = 1; k < QUEUE_MAX; k++) { queueTopic[k - 1] = queueTopic[k]; queueMsg[k - 1] = queueMsg[k]; }
    queueLen--;
  }
  queueTopic[queueLen] = t;
  queueMsg[queueLen] = msg;
  queueLen++;
}

void flushQueue() {
  int sent = 0;
  while (sent < queueLen && mqtt.connected()) {
    if (!mqtt.publish(queueTopic[sent].c_str(), queueMsg[sent].c_str())) break;
    sent++;
  }
  for (int k = sent; k < queueLen; k++) { queueTopic[k - sent] = queueTopic[k]; queueMsg[k - sent] = queueMsg[k]; }
  queueLen -= sent;
}

String readingsJson() {
  char buf[200];
  snprintf(buf, sizeof(buf),
           "\"v\":[%.1f,%.1f,%.1f],\"i\":[%.2f,%.2f,%.2f]",
           ph[0].v, ph[1].v, ph[2].v, ph[0].i, ph[1].i, ph[2].i);
  return String(buf);
}

// ==========================================================================
//  CẢNH BÁO
// ==========================================================================
void sendAlarm(const char* code, char phase, bool on, const char* msg) {
  char head[220];
  snprintf(head, sizeof(head),
           "{\"id\":\"%s\",\"code\":\"%s\",\"phase\":\"%c\",\"state\":\"%s\",\"ts\":%lld,\"msg\":\"%s\",",
           CABINET_ID, code, phase, on ? "on" : "off", nowEpoch(), msg);
  String json = String(head) + readingsJson() + "}";
  publishOrQueue(topic("alarm"), json);

  char line[200];
  snprintf(line, sizeof(line), "%lld,%s,%c,%s,%s", nowEpoch(), code, phase, on ? "on" : "off", msg);
  sdAppend("/alarms.csv", String(line));
  Serial.printf("[CANH BAO] %s %c %s - %s\n", code, phase, on ? "BAT" : "HET", msg);
}

// Bật cảnh báo khi điều kiện đúng "need" lần liên tiếp; tự báo hết khi điều kiện mất
void evalAlarm(AlarmId id, bool cond, uint8_t need, const char* msgOn, const char* msgOff) {
  Alarm& a = alarms[id];
  if (cond) {
    if (a.cnt < 255) a.cnt++;
    if (!a.active && a.cnt >= need) { a.active = true; sendAlarm(a.code, a.phase, true, msgOn); }
  } else {
    a.cnt = 0;
    if (a.active) { a.active = false; sendAlarm(a.code, a.phase, false, msgOff); }
  }
}

// ==========================================================================
//  ĐỌC CẢM BIẾN
// ==========================================================================
void readMeters() {
#if SIM_MODE
  bool mains = digitalRead(PIN_MAINS) == HIGH;
  for (int k = 0; k < 3; k++) {
    float noise = (random(-20, 21)) / 10.0;                // ±2V
    ph[k].v  = mains ? 225.0 + noise : 0.0;
    float amp = analogRead(PIN_POT[k]) * 10.0 / 4095.0;    // 0..10A
    ph[k].i  = mains ? amp : 0.0;
    ph[k].p  = ph[k].v * ph[k].i * 0.95;                   // PF ~0.95 (đèn LED)
    ph[k].wh += ph[k].p * READ_MS / 3600000.0;
    ph[k].ok = true;
  }
#else
  for (int k = 0; k < 3; k++) {
    uint8_t r = meter[k].readInputRegisters(0x0000, 10);   // bản đồ thanh ghi PZEM-016
    if (r == meter[k].ku8MBSuccess) {
      ph[k].v  = meter[k].getResponseBuffer(0) / 10.0;
      ph[k].i  = (meter[k].getResponseBuffer(1) | ((uint32_t)meter[k].getResponseBuffer(2) << 16)) / 1000.0;
      ph[k].p  = (meter[k].getResponseBuffer(3) | ((uint32_t)meter[k].getResponseBuffer(4) << 16)) / 10.0;
      ph[k].wh = (meter[k].getResponseBuffer(5) | ((uint32_t)meter[k].getResponseBuffer(6) << 16));
      ph[k].ok = true;
      meterFail[k] = 0;
    } else {
      ph[k].ok = false;
      if (meterFail[k] < 255) meterFail[k]++;
    }
  }
#endif
}

bool mpuReadAccel(float& ax, float& ay, float& az) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)0x68, (uint8_t)6) != 6) return false;
  int16_t x = (Wire.read() << 8) | Wire.read();
  int16_t y = (Wire.read() << 8) | Wire.read();
  int16_t z = (Wire.read() << 8) | Wire.read();
  ax = x / 16384.0; ay = y / 16384.0; az = z / 16384.0;    // thang ±2g
  return true;
}

void mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);                      // đánh thức MPU6050
  mpuOk = Wire.endTransmission() == 0;
  if (!mpuOk) { Serial.println("MPU6050: khong tim thay"); return; }
  delay(100);
  // Lấy vị trí "đứng thẳng" lúc lắp đặt làm mốc
  float sx = 0, sy = 0, sz = 0, ax, ay, az; int n = 0;
  for (int k = 0; k < 20; k++) {
    if (mpuReadAccel(ax, ay, az)) { sx += ax; sy += ay; sz += az; n++; }
    delay(10);
  }
  if (n > 0) { refAx = sx / n; refAy = sy / n; refAz = sz / n; }
  lastAx = refAx; lastAy = refAy; lastAz = refAz;
  Serial.printf("MPU6050 OK, moc: %.2f %.2f %.2f\n", refAx, refAy, refAz);
}

// Gọi liên tục (~50 lần/giây) để không bỏ sót cú va đập ngắn
void sampleMotion() {
  static uint32_t last = 0;
  if (!mpuOk || millis() - last < 20) return;
  last = millis();
  float ax, ay, az;
  if (!mpuReadAccel(ax, ay, az)) return;
  float d = sqrtf((ax - lastAx) * (ax - lastAx) + (ay - lastAy) * (ay - lastAy) + (az - lastAz) * (az - lastAz));
  if (d > vibPeak) vibPeak = d;
  lastAx = ax; lastAy = ay; lastAz = az;

  float m1 = sqrtf(ax * ax + ay * ay + az * az);
  float m0 = sqrtf(refAx * refAx + refAy * refAy + refAz * refAz);
  if (m1 > 0.1f && m0 > 0.1f) {
    float c = (ax * refAx + ay * refAy + az * refAz) / (m1 * m0);
    c = constrain(c, -1.0f, 1.0f);
    tiltDeg = acosf(c) * 180.0f / PI;
  }
}

// ==========================================================================
//  ĐIỀU KHIỂN RELAY THEO LỊCH
// ==========================================================================
bool scheduleWantsOn() {
  struct tm t;
  if (!timeValid() || !getLocalTime(&t, 0)) return true;   // chưa có giờ: ưu tiên SÁNG cho an toàn
  int now = t.tm_hour * 60 + t.tm_min;
  int on  = ON_HOUR * 60 + ON_MIN;
  int off = OFF_HOUR * 60 + OFF_MIN;
  return (on > off) ? (now >= on || now < off) : (now >= on && now < off);
}

void applyRelay() {
  bool want = (mode == MODE_ON) || (mode == MODE_AUTO && scheduleWantsOn());
  if (want != relayOn) {
    relayOn = want;
    relayChangedAt = millis();
    digitalWrite(PIN_RELAY, relayOn ? HIGH : LOW);
    for (int k = 0; k < 3; k++) phaseWasLoaded[k] = false;
    anyLoadSinceOn = false;
    Serial.printf("Relay: %s\n", relayOn ? "DONG (bat den)" : "CAT (tat den)");
  }
}

// ==========================================================================
//  PHÂN TÍCH SỰ CỐ
// ==========================================================================
void analyze() {
  bool settled = millis() - relayChangedAt > RELAY_GRACE_MS;

  // --- Điện lưới ---
  int lostCount = 0;
  for (int k = 0; k < 3; k++) if (ph[k].ok && ph[k].v < V_LOST) lostCount++;
  bool allLost = lostCount == 3;
  evalAlarm(AL_MAT_DIEN, allLost, DEBOUNCE_N, "Mat dien luoi ca 3 pha", "Da co dien lai");

  for (int k = 0; k < 3; k++) {
    bool lost = !allLost && ph[k].ok && ph[k].v < V_LOST;
    evalAlarm((AlarmId)(AL_MAT_PHA_A + k), lost, DEBOUNCE_N, "Mat dien ap pha", "Pha da co dien ap");
  }

  bool vBad = false;
  for (int k = 0; k < 3; k++)
    if (ph[k].ok && ph[k].v >= V_LOST && (ph[k].v < V_MIN_OK || ph[k].v > V_MAX_OK)) vBad = true;
  evalAlarm(AL_DIEN_AP, vBad, VOLT_DEBOUNCE_N, "Dien ap bat thuong (qua ap/sut ap)", "Dien ap binh thuong");

  // --- Dòng điện / cắt cáp ---
  for (int k = 0; k < 3; k++) {
    bool vOk = ph[k].ok && ph[k].v >= V_MIN_OK;
    if (relayOn && ph[k].ok && ph[k].i >= I_LOADED) { phaseWasLoaded[k] = true; anyLoadSinceOn = true; }
    // Pha từng có tải -> nay mất dòng, mà điện áp vẫn còn => đứt phía sau tủ: nghi cắt cáp
    bool cut = relayOn && phaseWasLoaded[k] && vOk && ph[k].i < I_ZERO;
    evalAlarm((AlarmId)(AL_CAT_CAP_A + k), cut, DEBOUNCE_N,
              "Pha dang tai bi mat dong, dien ap con - nghi cat/trom cap",
              "Pha da co dong tro lai");
  }

  float iTotal = ph[0].i + ph[1].i + ph[2].i;
  bool allVOk = ph[0].v >= V_MIN_OK && ph[1].v >= V_MIN_OK && ph[2].v >= V_MIN_OK;
  bool noLight = relayOn && settled && allVOk && !anyLoadSinceOn && iTotal < I_ZERO * 3;
  evalAlarm(AL_KHONG_SANG, noLight, DEBOUNCE_N,
            "Da dong relay nhung khong co dong - kiem tra contactor/CB", "Den da sang");

  bool dayLight = !relayOn && settled && iTotal >= I_LOADED;
  evalAlarm(AL_SANG_NGAY, dayLight, DEBOUNCE_N,
            "Relay da cat nhung van co dong - nghi dinh contactor", "Den da tat");

  // --- Va chạm / nghiêng ---
  if (mpuOk) {
    if (vibPeak > VIB_ALARM_G && millis() - lastCollisionAt > COLLISION_COOLDOWN_MS) {
      lastCollisionAt = millis();
      collisionEver = true;
      char msg[80];
      snprintf(msg, sizeof(msg), "Rung dong manh %.2fg - nghi xe va cham", vibPeak);
      sendAlarm("VA_CHAM", '-', true, msg);   // sự kiện tức thời
    }
    evalAlarm(AL_NGHIENG, tiltDeg > TILT_ALARM, DEBOUNCE_N, "Tu/tru bi nghieng", "Da het nghieng");
  }
  vibPeak = 0;

  // --- Cửa tủ ---
  bool doorOpen = digitalRead(PIN_DOOR) == HIGH;
  evalAlarm(AL_MO_CUA, doorOpen && !maintenance, 2, "Cua tu bi mo ngoai gio bao tri", "Cua tu da dong");

  // --- Lỗi đồng hồ ---
  bool meterErr = meterFail[0] >= 5 || meterFail[1] >= 5 || meterFail[2] >= 5;
  evalAlarm(AL_LOI_DONG_HO, meterErr, 1, "Khong doc duoc dong ho dien RS485", "Dong ho da phan hoi");
}

// ==========================================================================
//  GỬI DỮ LIỆU
// ==========================================================================
void sendTelemetry() {
  double wh = ph[0].wh + ph[1].wh + ph[2].wh;
  const char* modeStr = mode == MODE_AUTO ? "auto" : (mode == MODE_ON ? "on" : "off");
  char head[260];
  snprintf(head, sizeof(head),
           "{\"id\":\"%s\",\"ts\":%lld,\"p\":%.0f,\"kwh\":%.2f,\"relay\":%d,\"mode\":\"%s\","
           "\"maint\":%d,\"door\":%d,\"tilt\":%.1f,\"rssi\":%d,\"up\":%lu,\"q\":%d,\"sd\":%d,",
           CABINET_ID, nowEpoch(), ph[0].p + ph[1].p + ph[2].p, wh / 1000.0, relayOn ? 1 : 0, modeStr,
           maintenance ? 1 : 0, digitalRead(PIN_DOOR) == HIGH ? 1 : 0, tiltDeg, WiFi.RSSI(),
           (unsigned long)((millis() - bootMs) / 1000), queueLen, sdOk ? 1 : 0);
  String json = String(head) + readingsJson() + "}";
  if (mqtt.connected()) mqtt.publish(topic("telemetry").c_str(), json.c_str());
  Serial.println(json);
}

void logTelemetrySd() {
  char line[160];
  snprintf(line, sizeof(line), "%lld,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%d",
           nowEpoch(), ph[0].v, ph[1].v, ph[2].v, ph[0].i, ph[1].i, ph[2].i, relayOn ? 1 : 0);
  sdAppend("/tele.csv", String(line));
}

// ==========================================================================
//  LỆNH TỪ XA (topic .../cmd): on | off | auto | maint_on | maint_off | status
// ==========================================================================
void onMqttMessage(char* t, byte* payload, unsigned int len) {
  String cmd;
  for (unsigned int k = 0; k < len; k++) cmd += (char)payload[k];
  cmd.trim();
  Serial.printf("Lenh: %s\n", cmd.c_str());
  if      (cmd == "on")        mode = MODE_ON;
  else if (cmd == "off")       mode = MODE_OFF;
  else if (cmd == "auto")      mode = MODE_AUTO;
  else if (cmd == "maint_on")  maintenance = true;
  else if (cmd == "maint_off") maintenance = false;
  applyRelay();
  sendTelemetry();   // phản hồi ngay để app cập nhật
}

void ensureMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  if (millis() - lastMqttTry < 5000) return;
  lastMqttTry = millis();

  String clientId = String("tu-") + CABINET_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  String st = topic("status");
  // LWT: broker tự báo "offline" nếu thiết bị mất kết nối đột ngột
  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, st.c_str(), 1, true, "offline")) {
    Serial.println("MQTT: da ket noi");
    mqtt.publish(st.c_str(), "online", true);
    mqtt.subscribe(topic("cmd").c_str());
    flushQueue();
    sendTelemetry();
  } else {
    Serial.printf("MQTT: loi %d\n", mqtt.state());
  }
}

// ==========================================================================
void setup() {
  Serial.begin(115200);
  bootMs = millis();
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_DOOR, INPUT_PULLUP);
#if SIM_MODE
  pinMode(PIN_MAINS, INPUT_PULLUP);
  randomSeed(analogRead(39));
#else
  pinMode(PIN_RS485_DE, OUTPUT);
  rs485Rx();
  Serial2.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  for (int k = 0; k < 3; k++) {
    meter[k].begin(k + 1, Serial2);           // địa chỉ Modbus 1, 2, 3
    meter[k].preTransmission(rs485Tx);
    meter[k].postTransmission(rs485Rx);
  }
#endif
  for (int k = 0; k < 3; k++) ph[k] = {0, 0, 0, 0, false};

  mpuInit();
  sdOk = SD.begin(PIN_SD_CS);
  Serial.println(sdOk ? "SD: OK" : "SD: khong co the (bo qua ghi log)");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTzTime("ICT-7", "pool.ntp.org", "time.google.com");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);

  relayChangedAt = millis();
  applyRelay();
}

void loop() {
  ensureMqtt();
  mqtt.loop();
  sampleMotion();

  if (millis() - lastRead >= READ_MS) {
    lastRead = millis();
    applyRelay();
    readMeters();
    analyze();
  }
  if (millis() - lastTele >= TELEMETRY_MS) {
    lastTele = millis();
    sendTelemetry();
  }
  if (millis() - lastSdTele >= SD_TELE_MS) {
    lastSdTele = millis();
    logTelemetrySd();
  }
}
