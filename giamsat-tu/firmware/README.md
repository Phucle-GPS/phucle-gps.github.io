# Firmware gateway tủ chiếu sáng – ESP32

Firmware: `tu_chieu_sang/tu_chieu_sang.ino`
Trang mô phỏng + app giám sát: `../index.html` (chạy được ngay trên trình duyệt, không cần phần cứng)

## 1. Chạy thử trên Wokwi (không cần mua gì)

1. Vào https://wokwi.com → **New Project** → **ESP32**.
2. Dán nội dung `tu_chieu_sang.ino` vào tab `sketch.ino`.
3. Mở tab `diagram.json`, thay bằng nội dung file `diagram.json` trong thư mục này.
4. Bấm **Library Manager** → thêm `PubSubClient` (và `ModbusMaster`, hoặc dùng `libraries.txt`).
5. Sửa `TOPIC_ROOT` trong code thành chuỗi riêng, ví dụ `cs/doi-cs-7f3k2`.
6. Bấm ▶ chạy. Serial Monitor sẽ hiện `MQTT: da ket noi`.
7. Mở trang mô phỏng → tab **🧪 Giả lập** → mục *Kết nối thiết bị thật / Wokwi* → nhập cùng mã kênh → **Kết nối**.
   Tủ `TU-001` gắn nhãn **Thật** sẽ hiện trong danh sách.

Thử trên Wokwi:

| Thao tác trên mạch | Kết quả trên app |
|---|---|
| Vặn biến trở pha A lên ~3A, đợi vài giây, rồi vặn về 0 | **Nghi cắt / trộm cáp – pha A** |
| Gạt công tắc điện lưới (swMains) | **Mất điện lưới** (không báo nhầm thành cắt cáp) |
| Gạt công tắc cửa (swDoor) | **Mở cửa tủ trái phép** |
| Bấm vào MPU6050, kéo gia tốc thật nhanh | **Xe va chạm**; để lệch nhiều thì **Tủ/trụ bị nghiêng** |
| Trong app bấm 🔧 Bật chế độ bảo trì rồi mở cửa | Không báo |
| Trong app bấm ⚫ Tắt / 💡 Bật | LED relay trên mạch tắt/sáng |

> Nếu vừa chạy đã báo mất điện / mở cửa thì công tắc đang ở vị trí ngược: gạt lại.
> Ở SIM_MODE relay mặc định **bật cưỡng bức** để thử được bất kỳ giờ nào. Gửi lệnh "🕒 Theo lịch" để chạy theo giờ 18:00–05:30.

## 2. Phần cứng thật cho 1 tủ thử nghiệm

| Linh kiện | SL | Ghi chú |
|---|---|---|
| ESP32 DevKit (hoặc board ESP32 công nghiệp có sẵn RS485 + nguồn 220V) | 1 | |
| Đồng hồ PZEM-016 (Modbus RS485, 100A, kèm CT kẹp) | 3 | mỗi pha 1 cái, đặt địa chỉ 1, 2, 3 |
| Module chuyển TTL ↔ RS485 (MAX485 hoặc loại tự đảo chiều) | 1 | |
| Cảm biến MPU6050 | 1 | gắn chắc vào vỏ tủ |
| Công tắc từ (reed switch) cửa | 1 | |
| Module relay 1 kênh 5V (hoặc relay trung gian 24V) | 1 | đóng cuộn hút contactor |
| Module thẻ microSD + thẻ | 1 | |
| Nguồn 220VAC → 5VDC (HLK-PM05 hoặc nguồn ray DIN) | 1 | |
| Router 4G / module SIM (giai đoạn sau) | 1 | trong tủ thường không có Wi-Fi |

Giá tham khảo một bộ thử nghiệm khoảng 1–2 triệu đồng (ước lượng, cần kiểm tra lại giá khi mua).

**Đấu nối (khớp với code):**

| Tín hiệu | Chân ESP32 |
|---|---|
| RS485 RO / DI / DE+RE | GPIO16 / GPIO17 / GPIO4 |
| MPU6050 SDA / SCL | GPIO21 / GPIO22 |
| Relay contactor | GPIO26 |
| Reed switch cửa (đầu kia nối GND) | GPIO27 |
| SD: CS / SCK / MISO / MOSI | GPIO5 / 18 / 19 / 23 |

Rồi đổi `#define SIM_MODE 0`, điền Wi-Fi, broker riêng (có user/mật khẩu) và nạp.

> ⚠️ Phần đấu vào mạch 220V/380V của tủ (CT dòng, nguồn, cuộn hút contactor) phải do thợ điện có chuyên môn thực hiện, cắt điện trước khi làm và tuân thủ quy định an toàn của đơn vị. Nên đấu relay **song song** với timer hiện có ở giai đoạn thử nghiệm để nếu gateway hỏng thì đèn vẫn chạy theo timer cũ.

## 3. Định dạng bản tin MQTT

| Topic | Nội dung |
|---|---|
| `<ROOT>/tu/<ID>/telemetry` | `{"id","ts","v":[A,B,C],"i":[A,B,C],"p","kwh","relay","mode","maint","door","tilt","rssi","q","sd"}` mỗi 10 giây |
| `<ROOT>/tu/<ID>/alarm` | `{"id","code","phase","state":"on/off","ts","msg","v":[...],"i":[...]}` ngay khi có hoặc hết cảnh báo |
| `<ROOT>/tu/<ID>/status` | `online` / `offline` (retained, LWT) |
| `<ROOT>/tu/<ID>/cmd` | app gửi xuống: `on`, `off`, `auto`, `maint_on`, `maint_off`, `status` |

## 4. Logic phát hiện chính

- **Cắt/trộm cáp**: relay đang đóng, pha đó *đã từng có tải* (≥1A), nay dòng <0.2A **trong khi điện áp vẫn ≥180V** liên tục 3 giây. Còn điện áp nghĩa là đứt ở phía sau tủ, tức là trên tuyến cáp.
- **Mất điện lưới / mất pha**: điện áp <100V nên không báo nhầm thành cắt cáp.
- **Đèn không sáng**: đã đóng relay 30 giây, điện áp đủ, nhưng chưa từng có dòng nghĩa là hỏng contactor/CB.
- **Sáng ban ngày**: relay cắt mà vẫn có dòng nghĩa là dính tiếp điểm contactor.
- **Va chạm**: lấy mẫu MPU6050 50 lần/giây, gia tốc thay đổi >0.6g. **Nghiêng**: lệch >15° so với lúc lắp.
- **Mất mạng**: cảnh báo được giữ trong hàng đợi (20 bản tin) + ghi thẻ SD, có mạng lại thì gửi bù.

Các ngưỡng nằm ở đầu file `.ino`. Cần chỉnh theo tải thực tế từng tủ: tủ có ít đèn thì hạ `I_LOADED`.
