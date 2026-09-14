## 1. Hệ thống truyền động của bạn gồm 3 phần độc lập

| Phần | Điều khiển gì | Chân / thiết bị |
|---|---|---|
| **DAC MCP4725** | **Tốc độ** — xuất điện áp 0–5V analog cho driver động cơ | I2C (`Wire`), địa chỉ 0x60/0x61 |
| **Relay DIR** | **Chiều quay** — đảo cực `+`/`-` cấp cho động cơ | `PIN_DIR_L` (D6), `PIN_DIR_R` (D7) |
| **Relay BRAKE** | **Phanh** — đóng/nhả phanh cơ | `PIN_BRAKE_L` (D4), `PIN_BRAKE_R` (D5) |

Điểm mấu chốt: **tốc độ và chiều quay là hai thứ hoàn toàn tách biệt**. Muốn dừng bánh thì chỉ cần DAC về 0 (và/hoặc đóng phanh) — **không cần** đảo cực.

Đó chính là mấu chốt trả lời câu hỏi của bạn ở phần 4.

## 2. Vì sao "DAC chưa từng được ghi 0V" lại nghiêm trọng

Trong code có **hai tầng biến** cho tốc độ:

```
dac_out_L / dac_out_R   ← biến mềm (shadow), chỉ là con số trong RAM
        ↓  (chỉ được đồng bộ trong loop())
DAC vật lý (MCP4725)    ← điện áp THẬT SỰ ra driver động cơ
```

Việc ghi ra phần cứng **chỉ xảy ra** ở đầu [`loop()`](CONTROL%20WITH%20PLAYSTATION_SLAM/ARDUINO_MEGA/CONTROL_WITH_PLAYSTATION_SLAMSLAM_V2.ino:80):

```c
void loop() {
    if (dac_out_L != last_dac_L) {
        dac_R.setVoltage(dac_out_L, false);   // ← CHỈ Ở ĐÂY mới ghi I2C ra DAC
        last_dac_L = dac_out_L;
    }
    ...
```

Đây là một **tối ưu hoá** (tránh spam I2C mỗi vòng), nhưng nó **giả định `loop()` chạy liên tục**. Giả định đó bị phá vỡ bởi chính các hàm di chuyển.

### Bản gốc `BRAKE()` chỉ sửa biến RAM

```c
void BRAKE(){
    digitalWrite(PIN_BRAKE_L, LOW);
    digitalWrite(PIN_BRAKE_R, LOW);
    dac_out_L = 0;      // ← chỉ là phép gán biến C, KHÔNG có I2C traffic
    dac_out_R = 0;      // ← DAC vật lý vẫn đang xuất điện áp ~tốc độ đầy
}
```

### Và `DELAY()` chặn `loop()` chạy

Toàn bộ chuỗi đảo chiều chạy trong [`DI_TIEN()`](CONTROL%20WITH%20PLAYSTATION_SLAM/ARDUINO_MEGA/CONTROL_WITH_PLAYSTATION_SLAMSLAM_V2.ino:179):

```c
void DI_TIEN(float v_L, float v_R){
    BRAKE();                        // gán dac_out = 0 (nhưng không ghi DAC!)
    if (DIR_L == 1){ digitalWrite(PIN_DIR_L, HIGH); DELAY(1500); ... }  // ← 1.5 GIÂY
    if (DIR_R == 1){ digitalWrite(PIN_DIR_R, HIGH); DELAY(1500); ... }  // ← 1.5 GIÂY
    RE_BRAKE();
    ...
}
```

Và [`DELAY()`](CONTROL%20WITH%20PLAYSTATION_SLAM/ARDUINO_MEGA/CONTROL_WITH_PLAYSTATION_SLAMSLAM_V2.ino:222) chỉ gọi `DATA_ROS2()` — **không gọi `loop()`**:

```c
void DELAY(unsigned long wait_time){
    unsigned long start = millis();
    while (millis() - start < wait_time) {
        DATA_ROS2();   // chỉ in odometry, không cập nhật DAC
    }
}
```

### Kết quả

Trong suốt **~1.7 giây** đảo chiều (hoặc lên tới ~3.4s nếu phải đảo cả 2 bánh):

- `dac_out_L/R` trong RAM = 0 ✅
- **DAC vật lý vẫn xuất điện áp tốc độ đầy** ❌
- → **Động cơ vẫn được cấp điện đầy trong khi relay DIR đóng/cắt**
- → relay tiếp điểm ngắt dòng tải lớn → **hồ quang**
- → EMI đánh sập USB

Nói cách khác, cái mà bạn viết ra với ý định "ngắt điện rồi mới đảo relay" **chưa bao giờ thực sự ngắt điện**. Nó chỉ nói với chính nó rằng đã ngắt.

Thêm nữa, [`BRAKE()`](CONTROL%20WITH%20PLAYSTATION_SLAM/ARDUINO_MEGA/CONTROL_WITH_PLAYSTATION_SLAMSLAM_V2.ino:146) còn đóng **phanh cơ** trong khi motor vẫn đang đẩy hết công suất → phanh và motor "đấu" nhau.

## 3. Hậu quả vật lý khi đảo cực lúc motor còn quay

Đây là hiện tượng có tên riêng: **plugging** (hãm ngược / đảo cực cưỡng bức).

Khi motor quay, nó vừa là tải vừa là **máy phát**. Suất điện động cảm ứng (back-EMF):

```
E_back = Ke × ω
```

Lúc đảo cực, điện áp nguồn và back-EMF **cùng chiều về mặt dòng điện** → cộng dồn:

```
I_đảo = (V_nguồn + E_back) / R_armature
```

Ví dụ minh hoạ: nguồn 20V, back-EMF ~19V, `R_arm` ~1Ω → **I ≈ 39A tức thời**, so với dòng chạy bình thường ~2–4A. Đó là **xung dòng gấp ~10 lần**.

Xung dòng đó gây đồng thời 3 thứ:

1. **Hồ quang (arc) tại tiếp điểm relay** — không khí bị ion hoá, phóng tia lửa. Arc là nguồn phát **RF băng rộng** (broadband EMI), tức nhiễu đủ mọi tần số.
2. **di/dt khổng lồ** → bức xạ từ trường mạnh (near-field).
3. **Sụt áp rail 5V** do dòng inrush đập vào nguồn → chip USB ATmega16U2 của Mega (lấy nguồn từ rail này) glitch.

Nhiễu này phát vào cáp USB (dây D+/D- không chống nhiễu, chạy sát dây công suất) → tín hiệu vi sai USB méo → **hub xHCI của Pi 5 phát hiện sai và disable cổng**:

```
usb usb2-port2: disabled by hub (EMI?), re-enabling...
```

Nó giải thích luôn việc bạn thấy **"tạch tạch mấy lần mới ngắt"**: mỗi lần đảo relay, độ dài/mạnh của hồ quang **ngẫu nhiên** (phụ thuộc vị trí tiếp điểm, tốc độ motor lúc đó, thời điểm ngắt dòng AC/DC). Lần yếu thì không đủ gây lỗi, lần mạnh thì sập cổng.

## 4. Trả lời câu hỏi: "đang chạy mà relay đảo chiều thì đó là cần thiết để phanh bánh xong mới đảo hướng để xe rẽ an toàn chứ?"

**Bạn đúng về ý định — nhưng hiểu lầm về cơ chế.** Cần tách bạch hai việc:

### Phanh bánh ≠ đảo chiều

| Việc | Do ai làm | Cách làm đúng |
|---|---|---|
| **Phanh bánh** | Relay **BRAKE** + DAC | Kéo DAC về 0 (ngắt lực kéo), đóng phanh cơ |
| **Đảo chiều** | Relay **DIR** | **Chỉ được đóng/cắt khi motor đã dừng và đã mất điện** |

Việc **đảo cực relay** không phải là cách phanh — nó là **cách tạo hãm ngược** (plugging), một kỹ thuật thô bạo, phá hỏng motor, hộp số và tiếp điểm relay. Bạn **không cần** nó để dừng bánh. Bạn có relay BRAKE riêng để làm việc đó rồi.

### Ý định gốc của bạn trong code là ĐÚNG

Thứ tự bạn viết ra chính là thứ tự chuẩn công nghiệp:

```
1. BRAKE()          → ngắt lực kéo + đóng phanh
2. (chờ bánh dừng)  → dead-time
3. Đảo relay DIR    → lúc này motor đã mất điện, tiếp điểm đóng/cắt KHÔNG có dòng
4. RE_BRAKE()       → nhả phanh
5. Cấp tốc độ mới
```

Cái sai **không nằm ở logic**, mà ở hai chỗ thực thi:

- **(a) Bước 1 không ngắt điện thật** — bug shadow variable ở phần 2. Motor vẫn full công suất.
- **(b) Bước 2 không tồn tại** — không có khoảng chờ nào giữa "ngắt điện" và "đảo relay". Code nhảy thẳng từ `BRAKE()` sang `digitalWrite(PIN_DIR_L, HIGH)`.

Hệ quả: trên danh nghĩa là "phanh rồi mới đảo", thực tế thành **"đảo cực trong khi motor còn chạy full tốc độ"** — đúng tình huống plugging tệ nhất.

### Vậy "chạy mà đảo relay" có cần thiết không?

**Trong trường hợp lý tưởng thì không** — motor phải đứng yên trước khi đảo. Trên thực tế có hai tình huống:

- **Đổi chiều có chủ đích** (bạn bấm `F` rồi bấm `B`): hoàn toàn tránh được, vì phanh cơ + DAC=0 sẽ dừng bánh trong vài chục ms, rồi mới đảo. Không cần plugging.
- **Đảo chiều để hãm gấp / khẩn cấp**: người ta *có thể* dùng plugging có kiểm soát, nhưng khi đó phải thiết kế driver chuyên dụng (H-bridge có current limit, ramp, freewheel) chứ **không dùng relay cơ**. Relay cơ **không bao giờ** được phép đóng/cắt dưới tải cảm ứng lớn — đây là sai lầm thiết kế cổ điển.

Với hệ relay cơ của bạn, **nguyên tắc bắt buộc là: relay DIR chỉ được đóng/cắt khi dòng qua nó = 0**. Không có ngoại lệ.

## 5. Sau khi sửa, luồng chạy đúng

[`BRAKE()`](CONTROL%20WITH%20PLAYSTATION_SLAM/ARDUINO_MEGA/CONTROL_WITH_PLAYSTATION_SLAMSLAM_V2.ino:146) giờ ghi thẳng ra phần cứng, không chờ `loop()`:

```c
dac_out_L = 0;
dac_out_R = 0;
dac_R.setVoltage(0, false);   // ← I2C write NGAY, điện áp về 0 tức thì
dac_L.setVoltage(0, false);
```

và [`STOP_BEFORE_DIR_CHANGE()`](CONTROL%20WITH%20PLAYSTATION_SLAM/ARDUINO_MEGA/CONTROL_WITH_PLAYSTATION_SLAMSLAM_V2.ino:168) chèn khoảng chờ:

```c
void STOP_BEFORE_DIR_CHANGE(){
    BRAKE();                                    // 1. ngắt điện THẬT + đóng phanh
    DELAY(MOTOR_STOP_BEFORE_REVERSE_MS);        // 2. chờ 150ms cho dòng tắt
}
```

Chuỗi thực tế bây giờ:

```
đang chạy  →  DAC = 0V (I2C write ngay)  →  phanh đóng  →  chờ 150ms (dòng về 0)
           →  relay DIR đảo lúc dòng = 0 (không hồ quang)
           →  nhả phanh  →  loop() ghi tốc độ mới ra DAC  →  chạy chiều mới
```

Điểm cốt lõi: relay DIR bây giờ chuyển trạng thái **khi không có dòng** → không hồ quang → không EMI → cổng USB Pi sống sót.

## 6. Nếu vẫn còn ngắt sau khi test

Thứ tự tăng dần:

1. **Tăng `MOTOR_STOP_BEFORE_REVERSE_MS`** từ `150` lên `250` hoặc `300` — biến duy nhất cần chỉnh, ở đầu file.
2. **Tụ gốm 104 hàn trực tiếp giữa 2 cọc ra mỗi động cơ** — hấp thụ nhiễu tần cao từ chổi than. Tụ gốm **không phân cực** nên an toàn cả khi đảo chiều (khác hẳn diode, sẽ chập mạch khi đảo cực).
3. **Tụ 470µF + 0.1µF sát rail 5V Arduino** — quan trọng vì bạn đã bịt chân 5V USB, nên chip USB của Mega không còn tụ hold-up từ phía host nữa.
4. **Ferrite clamp quanh cáp USB** sát đầu Pi.
5. **Cân nhắc nối lại chân 5V USB** — đa số Mega có mạch auto-switch (MOSFET/diode) nên an toàn, và nó trả lại tụ bulk của host cho rail 5V.

Một lưu ý thiết kế dài hạn: vì mỗi lần đổi chiều tốn ~1.5s cho mỗi pulse relay cộng thêm dead-time, robot của bạn rất chậm khi rẽ. Giải pháp đúng về sau là thay relay cơ bằng **driver H-bridge có điều khiển PWM** (như BTS7960, VNH5019) — đảo chiều êm, không hồ quang, không EMI, và điều khiển được tốc độ liên tục. Nhưng đó là việc nâng cấp phần cứng, không cần cho việc sửa lỗi hiện tại.