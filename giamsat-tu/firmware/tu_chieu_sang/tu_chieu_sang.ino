/*
 * ============================================================================
 *  GATEWAY TỦ ĐIỀU KHIỂN CHIẾU SÁNG – ESP32            (giao thức v2)
 * ============================================================================
 *  GIÁM SÁT
 *   - Đo đếm 3 pha qua Modbus RTU RS485 (3 đồng hồ PZEM-016, địa chỉ 1-2-3):
 *     điện áp, dòng, công suất, hệ số công suất, tần số, điện năng kWh
 *   - 6 đầu vào số (DI, qua module cách ly quang):
 *       DI1 Cửa tủ   DI2 CB tổng   DI3 Phản hồi K1   DI4 Phản hồi K2
 *       DI5 Chống sét (SPD)        DI6 Công tắc Tay/Tự động tại tủ
 *   - Cảm biến rung/nghiêng MPU6050 (xe va chạm)
 *  ĐIỀU KHIỂN
 *   - 4 đầu ra relay (DO): K1 Cả đêm, K2 Nửa đêm, K3 Trang trí, K4 Dự phòng
 *   - Mỗi kênh có lịch bật/tắt riêng, lưu trong bộ nhớ (mất mạng vẫn chạy đúng lịch)
 *   - 2 chế độ: AUTO (theo lịch) / REMOTE (giữ theo lệnh từ app)
 *   - Công tắc tại tủ ở "Tay": từ chối mọi lệnh từ xa (an toàn khi đang sửa chữa)
 *  KẾT NỐI
 *   - MQTT: gửi đo lường, cảnh báo, phản hồi lệnh (ack); nhận lệnh dạng text
 *   - Hàng đợi cảnh báo khi mất mạng, ghi log thẻ SD
 *
 *  LỆNH (topic <ROOT>/tu/<ID>/cmd), dạng: "<ref> <lệnh> <tham số>"
 *     12 do 2 1                 bật kênh K2 (chuyển sang REMOTE)
 *     13 all 0                  tắt tất cả kênh (REMOTE)
 *     14 mode auto|remote       chế độ
 *     15 sched 2 1 18:00 23:00  lịch kênh 2: bật, 18:00 -> 23:00
 *     16 maint 1|0              chế độ bảo trì (không báo mở cửa)
 *     17 status                 gửi số liệu ngay
 *     18 sim cutA 1             (chỉ SIM_MODE) tạo lỗi giả lập
 *   Thiết bị trả lời ở topic .../ack: {"id","ref","ok","msg"}
 *
 *  SIM_MODE 1: chạy trên wokwi.com. Đo lường được tính bằng mô hình phần mềm
 *  (theo trạng thái DO), DI1/DI2/DI5/DI6 là công tắc gạt, DO là 4 đèn LED.
 *
 *  Thư viện: PubSubClient, ModbusMaster (chỉ khi SIM_MODE 0)
 * ============================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <math.h>

// ============================== CẤU HÌNH ==================================
#define SIM_MODE 1                          // 1 = giả lập Wokwi, 0 = phần cứng thật

const char* WIFI_SSID   = "Wokwi-GUEST";
const char* WIFI_PASS   = "";
const char* MQTT_HOST   = "broker.hivemq.com";   // broker công cộng – CHỈ để thử nghiệm
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER   = "";
const char* MQTT_PASS   = "";
const char* TOPIC_ROOT  = "cs/demo-doi-cs";      // trùng với Cài đặt > Kết nối trong app
const char* CABINET_ID  = "TU-001";

const int N_DO = 4, N_DI = 6;
const int PIN_DO[N_DO] = {25, 26, 27, 14};
const int PIN_DI[N_DI] = {34, 35, 36, 39, 32, 33};
const bool DI_INVERT   = false;              // đảo mức nếu module opto ra mức thấp khi có tín hiệu
enum { DI_DOOR = 0, DI_CB, DI_FB1, DI_FB2, DI_SPD, DI_AUTO };
const int FB_OF_DO[N_DO] = {DI_FB1, DI_FB2, -1, -1};   // kênh có tiếp điểm phụ phản hồi

// Lịch mặc định {bật, phút bật, phút tắt} – phút tính từ 00:00
struct Sched { uint8_t en; int16_t on; int16_t off; };
Sched sched[N_DO] = {
  {1, 18 * 60,      5 * 60 + 30},   // K1 Cả đêm   18:00 -> 05:30
  {1, 18 * 60,      23 * 60},       // K2 Nửa đêm  18:00 -> 23:00
  {1, 18 * 60 + 30, 22 * 60},       // K3 Trang trí 18:30 -> 22:00
  {0, 18 * 60,      5 * 60 + 30},   // K4 Dự phòng (tắt)
};

// Ngưỡng phát hiện
const float I_LOADED = 1.0, I_ZERO = 0.2;
const float V_LOST = 100, V_MIN_OK = 180, V_MAX_OK = 250;
const float TILT_ALARM = 15.0, VIB_ALARM_G = 0.6;
const uint8_t DEBOUNCE_N = 3, VOLT_DEBOUNCE_N = 10, FB_DEBOUNCE_N = 5;
const uint32_t GRACE_MS = 30000, READ_MS = 1000, TELEMETRY_MS = 10000, SD_TELE_MS = 60000;
const uint32_t COLLISION_COOLDOWN_MS = 60000;

const int PIN_SD_CS = 5, PIN_SDA = 21, PIN_SCL = 22;
const int PIN_RS485_RX = 16, PIN_RS485_TX = 17, PIN_RS485_DE = 4;

#if SIM_MODE
const float SIM_LOAD[3]    = {3.2, 2.9, 3.4};      // A khi tất cả kênh cùng sáng
const float SIM_SHARE[N_DO] = {0.6, 0.3, 0.1, 0.0}; // tỷ lệ tải của từng kênh
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
Preferences  prefs;

struct PhaseData { float v, i, p, pf; double wh; bool ok; };
PhaseData ph[3];
float freq = 0;

bool di[N_DI];
bool doState[N_DO];
bool remoteDo[N_DO];
enum Mode { MODE_AUTO, MODE_REMOTE };
Mode mode = MODE_AUTO;
bool maintenance = false;
uint32_t outputChangedAt = 0;
bool phaseWasLoaded[3] = {false, false, false};
bool anyLoadSinceChange = false;

#if SIM_MODE
struct SimFault { bool cut[3]; bool mains; bool kfail[N_DO]; bool kstuck[N_DO]; bool meter; } simF;
#endif

bool  mpuOk = false;
float refAx = 0, refAy = 0, refAz = 1, lastAx = 0, lastAy = 0, lastAz = 1;
float vibPeak = 0, tiltDeg = 0;
uint32_t lastCollisionAt = 0;
uint8_t meterFail[3] = {0, 0, 0};

bool sdOk = false;
const int QUEUE_MAX = 20;
String queueTopic[QUEUE_MAX], queueMsg[QUEUE_MAX];
int queueLen = 0;

enum AlarmId {
  AL_CAT_CAP_A, AL_CAT_CAP_B, AL_CAT_CAP_C, AL_MAT_PHA_A, AL_MAT_PHA_B, AL_MAT_PHA_C,
  AL_MAT_DIEN, AL_DIEN_AP, AL_CB_TONG, AL_KHONG_SANG, AL_SANG_NGAY,
  AL_K1_KHONG_DONG, AL_K2_KHONG_DONG, AL_K1_KHONG_CAT, AL_K2_KHONG_CAT,
  AL_SPD, AL_CHE_DO_TAY, AL_NGHIENG, AL_MO_CUA, AL_LOI_DONG_HO, AL_COUNT
};
struct Alarm { const char* code; char phase; uint8_t cnt; bool active; };
Alarm alarms[AL_COUNT] = {
  {"CAT_CAP", 'A', 0, false}, {"CAT_CAP", 'B', 0, false}, {"CAT_CAP", 'C', 0, false},
  {"MAT_PHA", 'A', 0, false}, {"MAT_PHA", 'B', 0, false}, {"MAT_PHA", 'C', 0, false},
  {"MAT_DIEN", '-', 0, false}, {"DIEN_AP", '-', 0, false}, {"CB_TONG", '-', 0, false},
  {"KHONG_SANG", '-', 0, false}, {"SANG_NGAY", '-', 0, false},
  {"K_KHONG_DONG", '1', 0, false}, {"K_KHONG_DONG", '2', 0, false},
  {"K_KHONG_CAT", '1', 0, false}, {"K_KHONG_CAT", '2', 0, false},
  {"SPD", '-', 0, false}, {"CHE_DO_TAY", '-', 0, false},
  {"NGHIENG", '-', 0, false}, {"MO_CUA", '-', 0, false}, {"LOI_DONG_HO", '-', 0, false},
};

uint32_t lastRead = 0, lastTele = 0, lastSdTele = 0, lastMqttTry = 0, bootMs = 0;

// ==========================================================================
//  TIỆN ÍCH
// ==========================================================================
String topic(const char* leaf) { return String(TOPIC_ROOT) + "/tu/" + CABINET_ID + "/" + leaf; }
bool timeValid() { return time(nullptr) > 1700000000; }
long long nowEpoch() { return timeValid() ? (long long)time(nullptr) : 0; }

void sdAppend(const char* path, const String& line) {
  if (!sdOk) return;
  File f = SD.open(path, FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

void publishOrQueue(const String& t, const String& msg) {
  if (mqtt.connected() && mqtt.publish(t.c_str(), msg.c_str())) return;
  if (queueLen >= QUEUE_MAX) {
    for (int k = 1; k < QUEUE_MAX; k++) { queueTopic[k - 1] = queueTopic[k]; queueMsg[k - 1] = queueMsg[k]; }
    queueLen--;
  }
  queueTopic[queueLen] = t; queueMsg[queueLen] = msg; queueLen++;
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

String arrF(float a, float b, float c, int dec) {
  char buf[64];
  if (dec == 1) snprintf(buf, sizeof(buf), "[%.1f,%.1f,%.1f]", a, b, c);
  else          snprintf(buf, sizeof(buf), "[%.2f,%.2f,%.2f]", a, b, c);
  return String(buf);
}

String readingsJson() {
  return String("\"v\":") + arrF(ph[0].v, ph[1].v, ph[2].v, 1) + ",\"i\":" + arrF(ph[0].i, ph[1].i, ph[2].i, 2);
}

// ==========================================================================
//  LỊCH + LƯU CẤU HÌNH
// ==========================================================================
void saveConfig() {
  prefs.putBytes("sched", sched, sizeof(sched));
  prefs.putUChar("mode", (uint8_t)mode);
  uint8_t r = 0;
  for (int c = 0; c < N_DO; c++) if (remoteDo[c]) r |= (1 << c);
  prefs.putUChar("rdo", r);
}

void loadConfig() {
  if (prefs.getBytesLength("sched") == sizeof(sched)) prefs.getBytes("sched", sched, sizeof(sched));
  mode = (Mode)prefs.getUChar("mode", MODE_AUTO);
  uint8_t r = prefs.getUChar("rdo", 0);
  for (int c = 0; c < N_DO; c++) remoteDo[c] = r & (1 << c);
}

bool schedOn(int ch) {
  struct tm t;
  if (!timeValid() || !getLocalTime(&t, 0)) return ch == 0;     // chưa có giờ: chỉ giữ K1 sáng (an toàn)
  const Sched& s = sched[ch];
  if (!s.en) return false;
  int now = t.tm_hour * 60 + t.tm_min;
  return (s.on > s.off) ? (now >= s.on || now < s.off) : (now >= s.on && now < s.off);
}

void applyOutputs() {
  bool changed = false;
  for (int c = 0; c < N_DO; c++) {
    bool want = (mode == MODE_AUTO) ? schedOn(c) : remoteDo[c];
    if (want != doState[c]) { doState[c] = want; changed = true; digitalWrite(PIN_DO[c], want ? HIGH : LOW); }
  }
  if (changed) {
    outputChangedAt = millis();
    for (int k = 0; k < 3; k++) phaseWasLoaded[k] = false;
    anyLoadSinceChange = false;
    Serial.printf("DO: %d%d%d%d\n", doState[0], doState[1], doState[2], doState[3]);
  }
}

// ==========================================================================
//  ĐỌC VÀO
// ==========================================================================
void readInputs() {
  for (int k = 0; k < N_DI; k++) di[k] = (digitalRead(PIN_DI[k]) == HIGH) != DI_INVERT;
#if SIM_MODE
  // Wokwi không có contactor: tiếp điểm phản hồi = trạng thái relay (trừ khi giả lập hỏng)
  for (int c = 0; c < N_DO; c++) {
    if (FB_OF_DO[c] < 0) continue;
    di[FB_OF_DO[c]] = (doState[c] || simF.kstuck[c]) && !simF.kfail[c];
  }
#endif
}

void readMeters() {
#if SIM_MODE
  bool powered = !simF.mains && di[DI_CB];          // mất lưới hoặc CB tổng cắt -> mất áp
  float share = 0;
  for (int c = 0; c < N_DO; c++) {
    bool closed = FB_OF_DO[c] >= 0 ? di[FB_OF_DO[c]] : doState[c];
    if (closed) share += SIM_SHARE[c];
  }
  for (int k = 0; k < 3; k++) {
    if (simF.meter) { ph[k].ok = false; if (meterFail[k] < 255) meterFail[k]++; continue; }
    meterFail[k] = 0;
    ph[k].ok = true;
    ph[k].v  = powered ? 225.0 + random(-20, 21) / 10.0 : 0.0;
    ph[k].i  = (powered && !simF.cut[k]) ? SIM_LOAD[k] * share * (1.0 + random(-30, 31) / 1000.0) : 0.0;
    ph[k].pf = ph[k].i > 0.1 ? 0.95 : 0.0;
    ph[k].p  = ph[k].v * ph[k].i * ph[k].pf;
    ph[k].wh += ph[k].p * READ_MS / 3600000.0;
  }
  freq = powered ? 50.0 + random(-5, 6) / 100.0 : 0;
#else
  for (int k = 0; k < 3; k++) {
    uint8_t r = meter[k].readInputRegisters(0x0000, 10);   // bản đồ thanh ghi PZEM-016
    if (r == meter[k].ku8MBSuccess) {
      ph[k].v  = meter[k].getResponseBuffer(0) / 10.0;
      ph[k].i  = (meter[k].getResponseBuffer(1) | ((uint32_t)meter[k].getResponseBuffer(2) << 16)) / 1000.0;
      ph[k].p  = (meter[k].getResponseBuffer(3) | ((uint32_t)meter[k].getResponseBuffer(4) << 16)) / 10.0;
      ph[k].wh = (meter[k].getResponseBuffer(5) | ((uint32_t)meter[k].getResponseBuffer(6) << 16));
      if (k == 0) freq = meter[k].getResponseBuffer(7) / 10.0;
      ph[k].pf = meter[k].getResponseBuffer(8) / 100.0;
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
  ax = x / 16384.0; ay = y / 16384.0; az = z / 16384.0;
  return true;
}

void mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);
  mpuOk = Wire.endTransmission() == 0;
  if (!mpuOk) { Serial.println("MPU6050: khong tim thay"); return; }
  delay(100);
  float sx = 0, sy = 0, sz = 0, ax, ay, az; int n = 0;
  for (int k = 0; k < 20; k++) { if (mpuReadAccel(ax, ay, az)) { sx += ax; sy += ay; sz += az; n++; } delay(10); }
  if (n > 0) { refAx = sx / n; refAy = sy / n; refAz = sz / n; }
  lastAx = refAx; lastAy = refAy; lastAz = refAz;
}

void sampleMotion() {
  static uint32_t last = 0;
  if (!mpuOk || millis() - last < 20) return;
  last = millis();
  float ax, ay, az;
  if (!mpuReadAccel(ax, ay, az)) return;
  float d = sqrtf((ax - lastAx) * (ax - lastAx) + (ay - lastAy) * (ay - lastAy) + (az - lastAz) * (az - lastAz));
  if (d > vibPeak) vibPeak = d;
  lastAx = ax; lastAy = ay; lastAz = az;
  float m1 = sqrtf(ax * ax + ay * ay + az * az), m0 = sqrtf(refAx * refAx + refAy * refAy + refAz * refAz);
  if (m1 > 0.1f && m0 > 0.1f) {
    float c = (ax * refAx + ay * refAy + az * refAz) / (m1 * m0);
    c = constrain(c, -1.0f, 1.0f);
    tiltDeg = acosf(c) * 180.0f / PI;
  }
}

// ==========================================================================
//  CẢNH BÁO
// ==========================================================================
void sendAlarm(const char* code, char phase, bool on, const char* msg) {
  char head[220];
  snprintf(head, sizeof(head),
           "{\"id\":\"%s\",\"code\":\"%s\",\"phase\":\"%c\",\"state\":\"%s\",\"ts\":%lld,\"msg\":\"%s\",",
           CABINET_ID, code, phase, on ? "on" : "off", nowEpoch(), msg);
  publishOrQueue(topic("alarm"), String(head) + readingsJson() + "}");
  char line[200];
  snprintf(line, sizeof(line), "%lld,%s,%c,%s,%s", nowEpoch(), code, phase, on ? "on" : "off", msg);
  sdAppend("/alarms.csv", String(line));
  Serial.printf("[CANH BAO] %s %c %s - %s\n", code, phase, on ? "BAT" : "HET", msg);
}

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

void analyze() {
  bool settled = millis() - outputChangedAt > GRACE_MS;
  bool localManual = !di[DI_AUTO];
  bool anyOn = false;
  for (int c = 0; c < N_DO; c++) anyOn |= doState[c];

  // --- Nguồn ---
  evalAlarm(AL_CB_TONG, !di[DI_CB], 2, "CB tong bi cat/nhay", "CB tong da dong");
  int lost = 0;
  for (int k = 0; k < 3; k++) if (ph[k].ok && ph[k].v < V_LOST) lost++;
  bool allLost = lost == 3;
  evalAlarm(AL_MAT_DIEN, allLost && di[DI_CB], DEBOUNCE_N, "Mat dien luoi ca 3 pha", "Da co dien lai");
  for (int k = 0; k < 3; k++)
    evalAlarm((AlarmId)(AL_MAT_PHA_A + k), !allLost && ph[k].ok && ph[k].v < V_LOST, DEBOUNCE_N, "Mat dien ap pha", "Pha da co dien ap");
  bool vBad = false;
  for (int k = 0; k < 3; k++)
    if (ph[k].ok && ph[k].v >= V_LOST && (ph[k].v < V_MIN_OK || ph[k].v > V_MAX_OK)) vBad = true;
  evalAlarm(AL_DIEN_AP, vBad, VOLT_DEBOUNCE_N, "Dien ap bat thuong", "Dien ap binh thuong");
  bool allVOk = ph[0].ok && ph[1].ok && ph[2].ok && ph[0].v >= V_MIN_OK && ph[1].v >= V_MIN_OK && ph[2].v >= V_MIN_OK;

  // --- Phản hồi contactor ---
  for (int c = 0; c < 2; c++) {
    int fb = FB_OF_DO[c];
    evalAlarm((AlarmId)(AL_K1_KHONG_DONG + c), !localManual && allVOk && doState[c] && !di[fb], FB_DEBOUNCE_N,
              "Da ra lenh dong nhung contactor khong dong", "Contactor da dong");
    evalAlarm((AlarmId)(AL_K1_KHONG_CAT + c), !localManual && !doState[c] && di[fb], FB_DEBOUNCE_N,
              "Da ra lenh cat nhung contactor van dong (dinh tiep diem)", "Contactor da cat");
  }

  // --- Dòng điện ---
  for (int k = 0; k < 3; k++)
    if (anyOn && ph[k].ok && ph[k].i >= I_LOADED) { phaseWasLoaded[k] = true; anyLoadSinceChange = true; }
  for (int k = 0; k < 3; k++) {
    bool vOk = ph[k].ok && ph[k].v >= V_MIN_OK;
    // Pha khác đang có tải mà pha này không có dòng (sau thời gian chờ) -> cáp pha này có thể đã bị cắt
    // từ trước khi bật đèn, hoặc bị cắt đúng lúc tủ chuyển kênh
    bool otherLoaded = false;
    for (int j = 0; j < 3; j++) if (j != k && ph[j].ok && ph[j].i >= I_LOADED) otherLoaded = true;
    bool dead = anyOn && vOk && ph[k].i < I_ZERO && (phaseWasLoaded[k] || (settled && otherLoaded));
    evalAlarm((AlarmId)(AL_CAT_CAP_A + k), dead, DEBOUNCE_N,
              "Pha khong co dong, dien ap con - nghi cat/trom cap", "Pha da co dong tro lai");
  }
  float iSum = ph[0].i + ph[1].i + ph[2].i;
  bool fbOk = !alarms[AL_K1_KHONG_DONG].active && !alarms[AL_K2_KHONG_DONG].active;
  evalAlarm(AL_KHONG_SANG, anyOn && settled && allVOk && fbOk && !anyLoadSinceChange && iSum < I_ZERO * 3, DEBOUNCE_N,
            "Contactor da dong nhung khong co dong - kiem tra CB nhanh/cap", "Den da sang");
  evalAlarm(AL_SANG_NGAY, !anyOn && !localManual && settled && iSum >= I_LOADED, DEBOUNCE_N,
            "Tat ca kenh da cat nhung van co dong", "Den da tat");

  // --- Thiết bị phụ ---
  evalAlarm(AL_SPD, !di[DI_SPD], 2, "Thiet bi chong set hong", "Chong set binh thuong");
  evalAlarm(AL_CHE_DO_TAY, localManual, 2, "Tu dang o che do Tay tai cho - khong dieu khien tu xa duoc", "Tu da ve che do Tu dong");
  evalAlarm(AL_MO_CUA, di[DI_DOOR] && !maintenance, 2, "Cua tu bi mo ngoai gio bao tri", "Cua tu da dong");
  bool meterErr = meterFail[0] >= 5 || meterFail[1] >= 5 || meterFail[2] >= 5;
  evalAlarm(AL_LOI_DONG_HO, meterErr, 1, "Khong doc duoc dong ho dien RS485", "Dong ho da phan hoi");

  if (mpuOk) {
    if (vibPeak > VIB_ALARM_G && millis() - lastCollisionAt > COLLISION_COOLDOWN_MS) {
      lastCollisionAt = millis();
      char msg[80];
      snprintf(msg, sizeof(msg), "Rung dong manh %.2fg - nghi xe va cham", vibPeak);
      sendAlarm("VA_CHAM", '-', true, msg);
    }
    evalAlarm(AL_NGHIENG, tiltDeg > TILT_ALARM, DEBOUNCE_N, "Tu/tru bi nghieng", "Da het nghieng");
  }
  vibPeak = 0;
}

// ==========================================================================
//  GỬI DỮ LIỆU
// ==========================================================================
void sendTelemetry() {
  double wh = ph[0].wh + ph[1].wh + ph[2].wh;
  char head[300];
  snprintf(head, sizeof(head),
           "{\"id\":\"%s\",\"ts\":%lld,\"f\":%.2f,\"kwh\":%.2f,\"mode\":\"%s\",\"maint\":%d,"
           "\"tilt\":%.1f,\"rssi\":%d,\"up\":%lu,\"q\":%d,\"sd\":%d,",
           CABINET_ID, nowEpoch(), freq, wh / 1000.0, mode == MODE_AUTO ? "auto" : "remote", maintenance ? 1 : 0,
           tiltDeg, WiFi.RSSI(), (unsigned long)((millis() - bootMs) / 1000), queueLen, sdOk ? 1 : 0);
  String j = String(head) + readingsJson();
  j += ",\"p\":" + arrF(ph[0].p, ph[1].p, ph[2].p, 1);
  j += ",\"pf\":" + arrF(ph[0].pf, ph[1].pf, ph[2].pf, 2);
  j += ",\"di\":[";
  for (int k = 0; k < N_DI; k++) { if (k) j += ","; j += di[k] ? "1" : "0"; }
  j += "],\"do\":[";
  for (int c = 0; c < N_DO; c++) { if (c) j += ","; j += doState[c] ? "1" : "0"; }
  j += "],\"sch\":[";
  for (int c = 0; c < N_DO; c++) {
    if (c) j += ",";
    j += "[" + String(sched[c].en) + "," + String(sched[c].on) + "," + String(sched[c].off) + "]";
  }
  j += "]}";
  if (mqtt.connected()) mqtt.publish(topic("telemetry").c_str(), j.c_str());
}

void logTelemetrySd() {
  char line[180];
  snprintf(line, sizeof(line), "%lld,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%d%d%d%d",
           nowEpoch(), ph[0].v, ph[1].v, ph[2].v, ph[0].i, ph[1].i, ph[2].i,
           doState[0], doState[1], doState[2], doState[3]);
  sdAppend("/tele.csv", String(line));
}

void sendAck(long ref, bool ok, const char* msg) {
  char buf[200];
  snprintf(buf, sizeof(buf), "{\"id\":\"%s\",\"ref\":%ld,\"ok\":%d,\"msg\":\"%s\"}", CABINET_ID, ref, ok ? 1 : 0, msg);
  if (mqtt.connected()) mqtt.publish(topic("ack").c_str(), buf);
}

// ==========================================================================
//  LỆNH TỪ XA
// ==========================================================================
void onMqttMessage(char* t, byte* payload, unsigned int len) {
  (void)t;
  char buf[100];
  if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
  memcpy(buf, payload, len);
  buf[len] = 0;
  Serial.printf("Lenh: %s\n", buf);

  long ref = 0; char cmd[12] = {0}; int n = 0;
  if (sscanf(buf, "%ld %11s %n", &ref, cmd, &n) < 2) { sendAck(0, false, "Sai cu phap"); return; }
  const char* args = buf + n;
  bool ok = true; const char* msg = "OK";
  bool remoteCmd = !strcmp(cmd, "do") || !strcmp(cmd, "all") || !strcmp(cmd, "mode");

  if (remoteCmd && !di[DI_AUTO]) {
    ok = false; msg = "Tu dang o che do Tay tai cho";
  } else if (!strcmp(cmd, "do") || !strcmp(cmd, "all")) {
    int ch = 0, val = 0;
    bool all = !strcmp(cmd, "all");
    bool parsed = all ? sscanf(args, "%d", &val) == 1 : (sscanf(args, "%d %d", &ch, &val) == 2 && ch >= 1 && ch <= N_DO);
    if (!parsed) { ok = false; msg = "Tham so sai"; }
    else {
      if (mode == MODE_AUTO) for (int c = 0; c < N_DO; c++) remoteDo[c] = doState[c];  // giữ nguyên các kênh khác
      mode = MODE_REMOTE;
      if (all) for (int c = 0; c < N_DO; c++) remoteDo[c] = val;
      else remoteDo[ch - 1] = val;
      saveConfig();
    }
  } else if (!strcmp(cmd, "mode")) {
    char m[10] = {0};
    sscanf(args, "%9s", m);
    if (!strcmp(m, "auto")) mode = MODE_AUTO;
    else if (!strcmp(m, "remote")) { for (int c = 0; c < N_DO; c++) remoteDo[c] = doState[c]; mode = MODE_REMOTE; }
    else { ok = false; msg = "Che do sai"; }
    saveConfig();
  } else if (!strcmp(cmd, "sched")) {
    int ch, en, h1, m1, h2, m2;
    if (sscanf(args, "%d %d %d:%d %d:%d", &ch, &en, &h1, &m1, &h2, &m2) == 6 && ch >= 1 && ch <= N_DO &&
        h1 >= 0 && h1 < 24 && m1 >= 0 && m1 < 60 && h2 >= 0 && h2 < 24 && m2 >= 0 && m2 < 60) {
      sched[ch - 1] = {(uint8_t)(en ? 1 : 0), (int16_t)(h1 * 60 + m1), (int16_t)(h2 * 60 + m2)};
      saveConfig();
    } else { ok = false; msg = "Lich sai"; }
  } else if (!strcmp(cmd, "maint")) {
    maintenance = atoi(args) != 0;
  } else if (!strcmp(cmd, "status")) {
    // chỉ gửi lại số liệu
#if SIM_MODE
  } else if (!strcmp(cmd, "sim")) {
    char name[12] = {0}; int val = 0;
    if (sscanf(args, "%11s %d", name, &val) == 2) {
      if (!strncmp(name, "cut", 3) && name[3] >= 'A' && name[3] <= 'C') simF.cut[name[3] - 'A'] = val;
      else if (!strcmp(name, "mains")) simF.mains = val;
      else if (!strcmp(name, "meter")) simF.meter = val;
      else if (!strncmp(name, "kfail", 5) && name[5] >= '1' && name[5] <= '4') simF.kfail[name[5] - '1'] = val;
      else if (!strncmp(name, "kstuck", 6) && name[6] >= '1' && name[6] <= '4') simF.kstuck[name[6] - '1'] = val;
      else { ok = false; msg = "Loi gia lap khong ro"; }
    } else { ok = false; msg = "Tham so sai"; }
#endif
  } else {
    ok = false; msg = "Lenh khong ho tro";
  }

  applyOutputs();
  sendAck(ref, ok, msg);
  sendTelemetry();
}

void ensureMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected() || millis() - lastMqttTry < 5000) return;
  lastMqttTry = millis();
  String clientId = String("tu-") + CABINET_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  String st = topic("status");
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
  for (int c = 0; c < N_DO; c++) { pinMode(PIN_DO[c], OUTPUT); digitalWrite(PIN_DO[c], LOW); doState[c] = false; remoteDo[c] = false; }
  for (int k = 0; k < N_DI; k++) pinMode(PIN_DI[k], INPUT);   // GPIO34-39 không có điện trở kéo: module opto cấp sẵn
#if SIM_MODE
  memset(&simF, 0, sizeof(simF));
  randomSeed(esp_random());
#else
  pinMode(PIN_RS485_DE, OUTPUT);
  rs485Rx();
  Serial2.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  for (int k = 0; k < 3; k++) {
    meter[k].begin(k + 1, Serial2);
    meter[k].preTransmission(rs485Tx);
    meter[k].postTransmission(rs485Rx);
  }
#endif
  for (int k = 0; k < 3; k++) ph[k] = {0, 0, 0, 0, 0, false};

  prefs.begin("tu", false);
  loadConfig();
  mpuInit();
  sdOk = SD.begin(PIN_SD_CS);
  Serial.println(sdOk ? "SD: OK" : "SD: khong co the");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTzTime("ICT-7", "pool.ntp.org", "time.google.com");
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(1024);

  outputChangedAt = millis();
  readInputs();
  applyOutputs();
}

void loop() {
  ensureMqtt();
  mqtt.loop();
  sampleMotion();
  if (millis() - lastRead >= READ_MS) {
    lastRead = millis();
    readInputs();
    applyOutputs();
    readMeters();
    analyze();
  }
  if (millis() - lastTele >= TELEMETRY_MS) { lastTele = millis(); sendTelemetry(); }
  if (millis() - lastSdTele >= SD_TELE_MS) { lastSdTele = millis(); logTelemetrySd(); }
}
