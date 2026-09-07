#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac_R;
Adafruit_MCP4725 dac_L;

// CẢM BIẾN HALL VÀ ĐỘNG CƠ (Giữ nguyên chuẩn chân theo Code 1)
const uint8_t PIN_HALL_L = 3; 
const uint8_t PIN_HALL_R = 2; 
const uint8_t PIN_BRAKE_L  = 4;                       
const uint8_t PIN_BRAKE_R  = 5;  
const uint8_t PIN_DIR_L   = 6;                       
const uint8_t PIN_DIR_R  = 7;  

volatile unsigned long tic_L = 0, tac_L = 0;
volatile unsigned long tic_R = 0, tac_R = 0;                                
volatile float feedback_L = 0;
volatile float feedback_R = 0;


uint32_t dac_out_L = 0;
uint32_t dac_out_R = 0;

// Biến tốc độ mặc định theo Code 2
double current_speed = 1037;
double current_speed_HM = 1200;

int DIR_L = 0;
int DIR_R = 0;

int HM_MODE = 0;

volatile long total_pulses_L = 0;
volatile long total_pulses_R = 0;


void setup() {
    Serial.begin(115200);   // Giao tiếp debug máy tính

    pinMode(PIN_HALL_L, INPUT);
    pinMode(PIN_HALL_R, INPUT);
    pinMode(PIN_BRAKE_L, OUTPUT);                           
    pinMode(PIN_BRAKE_R, OUTPUT);
    pinMode(PIN_DIR_L, OUTPUT);                           
    pinMode(PIN_DIR_R, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_HALL_L), intrupt_L, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_R), intrupt_R, CHANGE); 

    digitalWrite(PIN_BRAKE_L, LOW);
    digitalWrite(PIN_BRAKE_R, LOW);

    Wire.begin();
    if (!dac_R.begin(0x60)) { 
        Serial.println("Loi: Khong tim thay module MCP4725 Trai (0x60)!");
        while (1); 
    }
    
    if (!dac_L.begin(0x61)) { 
        Serial.println("Loi: Khong tim thay module MCP4725 Phai (0x61)!");
        while (1); 
    }
    dac_R.setVoltage(0, false); 
    dac_L.setVoltage(0, false);

    // Serial.println("=== HỆ THỐNG SẴN SÀNG! ĐỢI LỆNH TỪ ESP32... ===");
}

void loop() {
    // Xuất tín hiệu ra DAC liên tục
    dac_R.setVoltage(dac_out_L, false);
    dac_L.setVoltage(dac_out_R, false);

    DATA_ROS2();

    // In thông số feedback định kỳ ra màn hình Serial Monitor (giữ nguyên logic Code 1)
    // static unsigned long lastPrint = 0;
    // if (millis() - lastPrint > 1000) {
    //     lastPrint = millis();

        // if (HM_MODE == 1){Serial.print("HUMAN MODE");}
        // else {Serial.print("ROBOT MODE");}
        // Serial.print(" | feedback_L: ");
        // Serial.print(feedback_L, 0);
        // // Serial.print("  feedback_R: ");
        // Serial.print(feedback_R, 0);

        // // Serial.print(" | pulse left: ");
        // Serial.print(total_pulses_L);
        // // // Serial.print(" | pulse right: ");
        // Serial.print(total_pulses_R);

        // if (DIR_L == 0 && DIR_R == 0){
        //     Serial.print(" | ĐANG TIẾN");
        // }
        // else if (DIR_L == 1 && DIR_R == 1){
        //     Serial.print(" | ĐANG LÙI");
        // }
        // else if (DIR_L == 1 && DIR_R == 0){
        //     Serial.print(" | ĐANG CUA TRÁI");
        // }
        // else if (DIR_L == 0 && DIR_R == 1){
        //     Serial.print(" | ĐANG CUA PHẢI");
        // }
        // Serial.print(" | TỐC ĐỘ TRÁI: ");
        // Serial.print(dac_out_L);
        // // Serial.print(" | TỐC ĐỘ PHẢI: ");
        // Serial.println(dac_out_R);
    // }

    // Nhận lệnh điều khiển từ ESP32 chuyển sang qua Serial (Thay thế bàn phím bằng tay cầm)
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'F': // Tiến
                // Serial.println("Nhận lệnh: ĐI TIẾN");
                if (HM_MODE == 1){
                    DI_TIEN(current_speed_HM, current_speed_HM);
                } else {
                    DI_TIEN(current_speed, current_speed);
                }
                break;
            case 'B': // Lùi
                // Serial.println("Nhận lệnh: ĐI LÙI");
                if (HM_MODE == 1){
                    DI_LUI(current_speed_HM, current_speed_HM);
                } else {
                    DI_LUI(current_speed, current_speed);
                }
                break;
            case 'L': // Cua trái
                // Serial.println("Nhận lệnh: CUA TRÁI");
                if (HM_MODE == 1){
                    TURN_LEFT(current_speed_HM);
                } else {
                    TURN_LEFT(current_speed);
                }
                break;
            case 'R': // Cua phải
                // Serial.println("Nhận lệnh: CUA PHẢI");
                if (HM_MODE == 1){
                    TURN_RIGHT(current_speed_HM);
                } else {
                    TURN_RIGHT(current_speed);
                }
                break;
            case 'S': // Stop / Brake
                BRAKE();
                break;
            case 'X': // Nhả phanh / Rebrake
                RE_BRAKE();
                break;
            case 'Q': 
                TANG_TOC_PHAI();
                break;
            case 'A': 
                GIAM_TOC_PHAI();
                break;
            case 'E': 
                TANG_TOC_TRAI();
                break;
            case 'D': 
                GIAM_TOC_TRAI();
                break;
            case 'T': 
                ROBOT_MODE();
                break;
            case 'Y': 
                HUMAN_MODE();
                break;
        }
        
        while(Serial.available()) Serial.read(); // Xóa sạch bộ đệm
    }

    DELAY(20);
}

// HÀM NGẮT ĐỌC XUNG HALL (Giữ nguyên 100% Code 1)
void intrupt_L(){
    //ĐẾM XUNG PHẢI
    if (DIR_L == 0) {
        total_pulses_L++; 
    } else {
        total_pulses_L--; 
    }

    tic_L = millis();                                     
  
    if (tic_L - tac_L < 5){
        return;
    } else {  
        feedback_L = tic_L - tac_L;                                        
    }
    tac_L = tic_L; 
}

void intrupt_R(){
    //ĐẾM XUNG PHẢI
    if (DIR_R == 0) {
        total_pulses_R++; 
    } else {
        total_pulses_R--; 
    }

    tic_R = millis();                                     
  
    if (tic_R - tac_R < 5){
        return;
    } else {  
        feedback_R = tic_R - tac_R;                                        
    }
    tac_R = tic_R; 
}

// TOÀN BỘ CÁC HÀM XỬ LÝ ĐỘNG CƠ GIỮ NGUYÊN 100% LOGIC CODE 1
void BRAKE(){
    digitalWrite(PIN_BRAKE_L, LOW);
    digitalWrite(PIN_BRAKE_R, LOW);

    dac_out_L = 0;
    dac_out_R = 0;
    dac_R.setVoltage(0, false);
    dac_L.setVoltage(0, false);

    // Serial.println("=== BRAKE ===");
}

void RE_BRAKE(){
    digitalWrite(PIN_BRAKE_L, HIGH);
    digitalWrite(PIN_BRAKE_R, HIGH);
    // Serial.println("=== RE_BRAKE ===");
}

void DI_CHUYEN (float v_L, float v_R){
    RE_BRAKE();

    dac_out_L = v_L;
    dac_out_R = v_R;

    // Serial.print("TỐC ĐỘ DI CHUYỂN: TRÁI=  ");
    // Serial.print(dac_out_L);
    // // Serial.print(" | Phai= ");
    // Serial.println(dac_out_R);
}

void DI_TIEN(float v_L, float v_R){
    BRAKE();

    if (DIR_L == 1){
        digitalWrite(PIN_DIR_L, HIGH);
        // Serial.println("=== ĐANG ĐẢO CHIỀU ===");
        DELAY(1500);

        digitalWrite(PIN_DIR_L, LOW);
        DELAY(200);
    }
    if (DIR_R == 1){
        digitalWrite(PIN_DIR_R, HIGH);
        // Serial.println("=== ĐANG ĐẢO CHIỀU ===");
        DELAY(1500);

        digitalWrite(PIN_DIR_R, LOW);
        DELAY(200);
    }

    // Serial.println("=== ĐẢO CHIỀU HOÀN TẤT ===");

    RE_BRAKE();

    DI_CHUYEN(v_L, v_R);

    DIR_L = 0;
    DIR_R = 0;
}

void DI_LUI(float v_L, float v_R){
    BRAKE();

    if (DIR_L == 0){
        digitalWrite(PIN_DIR_L, HIGH);
        // Serial.println("=== ĐANG ĐẢO CHIỀU ===");
        DELAY(1500);

        digitalWrite(PIN_DIR_L, LOW);
        DELAY(200);
    }
    if (DIR_R == 0){
        digitalWrite(PIN_DIR_R, HIGH);
        // Serial.println("=== ĐANG ĐẢO CHIỀU ===");
        DELAY(1500);

        digitalWrite(PIN_DIR_R, LOW);
        DELAY(200);
    }

    // Serial.println("=== ĐẢO CHIỀU HOÀN TẤT ===");

    RE_BRAKE();

    DI_CHUYEN(v_L, v_R);

    DIR_L = 1;
    DIR_R = 1;
}

void TURN_LEFT(float v){
    BRAKE();

    if (DIR_L == 0){
        digitalWrite(PIN_DIR_L, HIGH);
        // Serial.println("=== ĐANG ĐẢO CHIỀU CUA TRÁI ===");
        DELAY(1500);

        digitalWrite(PIN_DIR_L, LOW);
        DELAY(200);
    }
    if (DIR_R == 1){
        digitalWrite(PIN_DIR_R, HIGH);
        // Serial.println("=== ĐANG ĐẢO CHIỀU CUA TRÁI ===");
        DELAY(1500);

        digitalWrite(PIN_DIR_R, LOW);
        DELAY(200);
    }

    // Serial.println("=== CUA TRÁI HOÀN TẤT ===");

    RE_BRAKE();

    DI_CHUYEN(v, v);

    DIR_L = 1;
    DIR_R = 0;
}

void TURN_RIGHT(float v){
    BRAKE();

    if (DIR_L == 1){
        digitalWrite(PIN_DIR_L, HIGH);
        // Serial.println("=== ĐANG ĐẢO CHIỀU CUA PHẢI ===");
        DELAY(1500);

        digitalWrite(PIN_DIR_L, LOW);
        DELAY(200);
    }
    if (DIR_R == 0){
        digitalWrite(PIN_DIR_R, HIGH);
        // Serial.println("=== ĐANG ĐẢO CHIỀU CUA PHẢI ===");
        DELAY(1500);

        digitalWrite(PIN_DIR_R, LOW);
        DELAY(200);
    }
    
    // Serial.println("=== CUA PHẢI HOÀN TẤT ===");

    RE_BRAKE();

    DI_CHUYEN(v, v);

    DIR_L = 0;
    DIR_R = 1;
}

void TANG_TOC_TRAI(){
    dac_out_L +=5;
}

void GIAM_TOC_TRAI(){
    dac_out_L -=5;
}

void TANG_TOC_PHAI(){
    dac_out_R +=5;
}

void GIAM_TOC_PHAI(){
    dac_out_R -=5;
}

void HUMAN_MODE(){
    HM_MODE = 1;
    // Serial.println("=== HUMAN MODE ===");
}

void ROBOT_MODE(){
    HM_MODE = 0;
    // Serial.println("=== ROBOT MODE ===");
}

void DELAY(unsigned long wait_time){
    unsigned long start = millis();
    while (millis() - start < wait_time) {
        DATA_ROS2(); 
    }
}

void DATA_ROS2() {
    static unsigned long last_ros_print = 0;
    // Bơm dữ liệu liên tục 20 lần/giây (mỗi 50ms)
    if (millis() - last_ros_print >= 50) {
        long pulse_L = 0, pulse_R = 0;
        
        noInterrupts();
        pulse_L = total_pulses_L;
        pulse_R = total_pulses_R;
        interrupts();
        
        // Gửi qua cổng Serial (Cáp USB cắm vào máy tính ROS 2)
        Serial.print(pulse_L);
        Serial.print(",");
        Serial.println(pulse_R);
        
        last_ros_print = millis();
    }
}