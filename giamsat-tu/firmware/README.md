# Firmware gateway tủ điều khiển chiếu sáng – ESP32 (giao thức v2)

- Firmware: `tu_chieu_sang/tu_chieu_sang.ino`
- App điều khiển (điện thoại + máy tính): `../index.html`, mở tại `https://phucle-gps.github.io/giamsat-tu/`

## Tủ làm được gì

| Nhóm | Chi tiết |
|---|---|
| **Đo đếm** | Điện áp, dòng, công suất, cos φ từng pha; tần số; chỉ số kWh (3 đồng hồ PZEM-016 qua RS485) |
| **Đầu vào (DI)** | DI1 cửa tủ · DI2 CB tổng · DI3 phản hồi K1 · DI4 phản hồi K2 · DI5 chống sét · DI6 công tắc Tay/Tự động |
| **Đầu ra (DO)** | K1 Cả đêm · K2 Nửa đêm · K3 Trang trí · K4 Dự phòng, mỗi kênh có lịch riêng |
| **Điều khiển** | Tự động theo lịch (lưu trong tủ, mất mạng vẫn chạy), hoặc điều khiển từ xa qua app; công tắc tại tủ để Tay thì app không điều khiển được |
| **Cảnh báo** | Cắt/trộm cáp, mất điện, mất pha, CB tổng nhảy, contactor không đóng/dính, đèn không sáng, sáng ngoài lịch, mở cửa, chống sét hỏng, xe va chạm, nghiêng, lỗi đồng hồ |

## 1. Chạy thử trên Wokwi (không cần phần cứng)

1. Vào https://wokwi.com → **New Project** → **ESP32**.
2. Dán `tu_chieu_sang.ino` vào `sketch.ino`, dán `diagram.json` vào tab `diagram.json`.
3. Library Manager: thêm `PubSubClient` (và `ModbusMaster`).
4. Đổi `TOPIC_ROOT` thành chuỗi riêng, khó đoán, ví dụ `cs/doi-cs-7f3k2`.
5. Bấm ▶. Serial Monitor hiện `MQTT: da ket noi`.
6. Mở app → **Cài đặt** → nhập cùng *Mã kênh* → **Lưu & kết nối**. Tủ `TU-001` gắn nhãn **Thật** sẽ xuất hiện.

| Trên mạch Wokwi | App hiển thị |
|---|---|
| 4 đèn LED | Trạng thái K1–K4; bấm công tắc trong app → đèn LED đổi theo |
| Gạt công tắc **DI1 Cửa** | DI1 = Mở → cảnh báo **Mở cửa tủ trái phép** (trừ khi bật Bảo trì) |
| Gạt **DI2 CB** | **CB tổng bị cắt/nhảy**, mất áp |
| Gạt **DI5 SPD** | **Chống sét hỏng** |
| Gạt **DI6 Auto** | **Tủ ở chế độ Tay**: lệnh từ app bị từ chối |
| Kéo gia tốc MPU6050 | **Xe va chạm** / **Tủ bị nghiêng** |

Lỗi điện không có trên Wokwi (cắt cáp, mất lưới, contactor hỏng) có thể tạo bằng lệnh `sim` (mục 4), ví dụ gửi `1 sim cutA 1`.

> Nếu vừa chạy đã báo mở cửa / CB cắt thì công tắc đang ở vị trí ngược: gạt lại.

## 2. Phần cứng cho 1 tủ

| Linh kiện | SL | Ghi chú |
|---|---|---|
| ESP32 DevKit, hoặc board ESP32 công nghiệp có sẵn RS485, relay, opto | 1 | board công nghiệp gọn và bền hơn trong tủ |
| Đồng hồ PZEM-016 (RS485, kèm CT kẹp 100A) | 3 | mỗi pha 1 cái, đặt địa chỉ Modbus 1, 2, 3 |
| Module TTL ↔ RS485 (MAX485 / tự đảo chiều) | 1 | |
| Module cách ly quang 220VAC (hoặc 24VDC) → 3.3V, 6–8 kênh | 1 | đọc DI: tiếp điểm cửa, CB, phụ contactor, SPD, công tắc Tay/Tự động |
| Module relay 4 kênh (có opto) | 1 | đóng cuộn hút contactor K1–K4 qua relay trung gian |
| Công tắc cửa (hành trình / từ) | 1 | |
| MPU6050 | 1 | gắn chắc vào vỏ tủ |
| Module microSD + thẻ | 1 | |
| Nguồn 220VAC → 5VDC ray DIN | 1 | |
| Router 4G nhỏ, hoặc module SIM 4G | 1 | trong tủ thường không có Wi-Fi |

Giá tham khảo khoảng 1,5–3 triệu đồng/tủ cho bản thử nghiệm (ước lượng, cần kiểm tra lại khi mua).

**Đấu nối (khớp với code):**

| Tín hiệu | Chân ESP32 |
|---|---|
| DO1–DO4 (relay K1–K4) | GPIO25, 26, 27, 14 |
| DI1–DI6 (ra của module opto) | GPIO34, 35, 36, 39, 32, 33 |
| RS485 RO / DI / DE+RE | GPIO16 / 17 / 4 |
| MPU6050 SDA / SCL | GPIO21 / 22 |
| SD: CS / SCK / MISO / MOSI | GPIO5 / 18 / 19 / 23 |

GPIO34–39 không có điện trở kéo trong; module opto thường có sẵn. Nếu module ra mức thấp khi có tín hiệu, đặt `DI_INVERT = true`.
Sau đó đổi `#define SIM_MODE 0`, điền Wi-Fi, broker riêng có user/mật khẩu rồi nạp.

> ⚠️ **An toàn**: phần đấu vào mạch 220/380V của tủ (CT, nguồn, cuộn hút, tiếp điểm phụ) phải do thợ điện có chuyên môn làm, cắt điện trước khi thao tác, theo quy định của đơn vị.
> Giai đoạn thử nghiệm nên giữ timer/công tắc Tay hiện có: gateway hỏng thì tủ vẫn chạy bằng mạch cũ. Công tắc Tay/Tự động (DI6) phải là công tắc cứng trong mạch điều khiển, không chỉ là tín hiệu báo.

## 3. Bản tin MQTT

| Topic | Hướng | Nội dung |
|---|---|---|
| `<ROOT>/tu/<ID>/telemetry` | tủ → app, 10 giây/lần | `{"id","ts","v":[3],"i":[3],"p":[3],"pf":[3],"f","kwh","di":[6],"do":[4],"sch":[[en,onMin,offMin]×4],"mode":"auto/remote","maint","tilt","rssi","up","q","sd"}` |
| `<ROOT>/tu/<ID>/alarm` | tủ → app | `{"id","code","phase","state":"on/off","ts","msg","v","i"}` (`phase` là A/B/C, hoặc số kênh 1/2 với lỗi contactor) |
| `<ROOT>/tu/<ID>/ack` | tủ → app | `{"id","ref","ok","msg"}` phản hồi cho từng lệnh |
| `<ROOT>/tu/<ID>/status` | broker | `online` / `offline` (retained, LWT) |
| `<ROOT>/tu/<ID>/cmd` | app → tủ | văn bản `"<ref> <lệnh> <tham số>"` |

## 4. Lệnh

| Lệnh | Ý nghĩa |
|---|---|
| `12 do 2 1` | Bật K2 (tủ chuyển sang điều khiển từ xa, các kênh khác giữ nguyên) |
| `13 all 0` | Tắt tất cả kênh |
| `14 mode auto` | Về chạy theo lịch |
| `15 sched 2 1 18:00 23:00` | Lịch K2: bật lúc 18:00, tắt lúc 23:00 (lưu vào bộ nhớ tủ) |
| `16 maint 1` | Bật chế độ bảo trì (không báo mở cửa) |
| `17 status` | Gửi số liệu ngay |
| `18 sim cutA 1` | *(chỉ SIM_MODE)* tạo lỗi: `cutA/B/C`, `mains`, `kfail1..4`, `kstuck1..4`, `meter` |

App chờ `ack` tối đa 10 giây. Không có phản hồi thì báo "Tủ không phản hồi", không âm thầm coi như đã làm.

## 5. Logic phát hiện chính

- **Cắt/trộm cáp**: có kênh đang đóng, điện áp pha vẫn ≥180V nhưng dòng <0.2A suốt 3 giây, khi pha đó đã từng có tải **hoặc** (sau 30 giây khởi động) các pha khác đang có tải. Vế sau bắt được trường hợp cáp bị cắt ban ngày, tối bật đèn mới lộ, hoặc bị cắt đúng lúc tủ chuyển kênh.
  > Nếu tủ thiết kế có pha không mang tải thì cần tắt kiểm tra cho pha đó, nếu không sẽ báo nhầm.
- **Mất điện lưới**: cả 3 pha <100V mà CB tổng vẫn đóng. **CB tổng nhảy** thì báo riêng.
- **Contactor không đóng / dính**: lệnh DO và tiếp điểm phản hồi DI lệch nhau 5 giây.
- **Đèn không sáng**: contactor đã đóng, có điện áp, nhưng sau 30 giây không có dòng (đứt CB nhánh, cầu chì, cáp).
- **Sáng ngoài lịch**: mọi kênh đã cắt mà vẫn có dòng.
- **Mất mạng**: cảnh báo giữ trong hàng đợi (20 bản tin) + ghi thẻ SD, có mạng lại thì gửi bù. Lịch vẫn chạy bằng giờ NTP đã đồng bộ.

Ngưỡng nằm ở đầu file `.ino`, cần chỉnh theo tải từng tủ. App dùng cùng ngưỡng ở mục `CFG`.
