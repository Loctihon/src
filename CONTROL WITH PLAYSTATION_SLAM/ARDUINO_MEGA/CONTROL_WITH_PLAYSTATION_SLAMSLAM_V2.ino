#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac_R;
Adafruit_MCP4725 dac_L;

// CẢM BIẾN HALL VÀ ĐỘNG CƠ
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

// Biến lưu trạng thái I2C để chống treo mạch
uint32_t last_dac_L = 1;
uint32_t last_dac_R = 1;

// Biến tốc độ mặc định
double current_speed = 1045;
double current_speed_HM = 1200;

int DIR_L = 0;
int DIR_R = 0;
int HM_MODE = 0;

volatile long total_pulses_L = 0;
volatile long total_pulses_R = 0;

void setup() {
    Serial.begin(115200);   

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
    // Tăng tốc độ I2C lên 400kHz để giao tiếp nhanh hơn
    Wire.setClock(400000); 

    if (!dac_R.begin(0x60)) { 
        Serial.println("Loi: Khong tim thay MCP4725 Trai (0x60)!");
        while (1); 
    }
    
    if (!dac_L.begin(0x61)) { 
        Serial.println("Loi: Khong tim thay MCP4725 Phai (0x61)!");
        while (1); 
    }
    dac_R.setVoltage(0, false); 
    dac_L.setVoltage(0, false);
}

void loop() {
    // 1. CHỐNG TREO I2C: Chỉ xuất tín hiệu ra DAC khi tốc độ thực sự thay đổi
    if (dac_out_L != last_dac_L) {
        dac_R.setVoltage(dac_out_L, false);
        last_dac_L = dac_out_L;
    }
    if (dac_out_R != last_dac_R) {
        dac_L.setVoltage(dac_out_R, false);
        last_dac_R = dac_out_R;
    }

    DATA_ROS2();

    // 2. VÉT BỘ ĐỆM: Đọc liên tục để loại bỏ lệnh cũ, CHỈ thực thi lệnh cuối cùng
    if (Serial.available() > 0) {
        char cmd = 'S';
        while (Serial.available() > 0) {
            char temp = Serial.read();
            if (temp != '\n' && temp != '\r' && temp != ' ') {
                cmd = temp; 
            }
        }
        
        switch (cmd) {
            case 'F': 
                if (HM_MODE == 1) DI_TIEN(current_speed_HM, current_speed_HM);
                else DI_TIEN(current_speed, current_speed);
                break;
            case 'B': 
                if (HM_MODE == 1) DI_LUI(current_speed_HM, current_speed_HM);
                else DI_LUI(current_speed, current_speed);
                break;
            case 'L': 
                if (HM_MODE == 1) TURN_LEFT(current_speed_HM);
                else TURN_LEFT(current_speed);
                break;
            case 'R': 
                if (HM_MODE == 1) TURN_RIGHT(current_speed_HM);
                else TURN_RIGHT(current_speed);
                break;
            case 'S': BRAKE(); break;
            case 'X': RE_BRAKE(); break;
            case 'Q': TANG_TOC_PHAI(); break;
            case 'A': GIAM_TOC_PHAI(); break;
            case 'E': TANG_TOC_TRAI(); break;
            case 'D': GIAM_TOC_TRAI(); break;
            case 'T': ROBOT_MODE(); break;
            case 'Y': HUMAN_MODE(); break;
        }
    }
}

// ================= CÁC HÀM XỬ LÝ (GIỮ NGUYÊN LOGIC CỦA BẠN) =================

void intrupt_L(){
    if (DIR_L == 0) total_pulses_L++; else total_pulses_L--; 
    tic_L = millis();                                     
    if (tic_L - tac_L >= 5) feedback_L = tic_L - tac_L;                                        
    tac_L = tic_L; 
}

void intrupt_R(){
    if (DIR_R == 0) total_pulses_R++; else total_pulses_R--; 
    tic_R = millis();                                     
    if (tic_R - tac_R >= 5) feedback_R = tic_R - tac_R;                                        
    tac_R = tic_R; 
}

void BRAKE(){
    digitalWrite(PIN_BRAKE_L, LOW);
    digitalWrite(PIN_BRAKE_R, LOW);
    dac_out_L = 0;
    dac_out_R = 0;
}

void RE_BRAKE(){
    digitalWrite(PIN_BRAKE_L, HIGH);
    digitalWrite(PIN_BRAKE_R, HIGH);
}

void DI_CHUYEN (float v_L, float v_R){
    RE_BRAKE();
    dac_out_L = v_L;
    dac_out_R = v_R;
}

void DI_TIEN(float v_L, float v_R){
    BRAKE();
    if (DIR_L == 1){ digitalWrite(PIN_DIR_L, HIGH); DELAY(1500); digitalWrite(PIN_DIR_L, LOW); DELAY(200); }
    if (DIR_R == 1){ digitalWrite(PIN_DIR_R, HIGH); DELAY(1500); digitalWrite(PIN_DIR_R, LOW); DELAY(200); }
    RE_BRAKE();
    DI_CHUYEN(v_L, v_R);
    DIR_L = 0; DIR_R = 0;
}

void DI_LUI(float v_L, float v_R){
    BRAKE();
    if (DIR_L == 0){ digitalWrite(PIN_DIR_L, HIGH); DELAY(1500); digitalWrite(PIN_DIR_L, LOW); DELAY(200); }
    if (DIR_R == 0){ digitalWrite(PIN_DIR_R, HIGH); DELAY(1500); digitalWrite(PIN_DIR_R, LOW); DELAY(200); }
    RE_BRAKE();
    DI_CHUYEN(v_L, v_R);
    DIR_L = 1; DIR_R = 1;
}

void TURN_LEFT(float v){
    BRAKE();
    if (DIR_L == 0){ digitalWrite(PIN_DIR_L, HIGH); DELAY(1500); digitalWrite(PIN_DIR_L, LOW); DELAY(200); }
    if (DIR_R == 1){ digitalWrite(PIN_DIR_R, HIGH); DELAY(1500); digitalWrite(PIN_DIR_R, LOW); DELAY(200); }
    RE_BRAKE();
    DI_CHUYEN(v, v);
    DIR_L = 1; DIR_R = 0;
}

void TURN_RIGHT(float v){
    BRAKE();
    if (DIR_L == 1){ digitalWrite(PIN_DIR_L, HIGH); DELAY(1500); digitalWrite(PIN_DIR_L, LOW); DELAY(200); }
    if (DIR_R == 0){ digitalWrite(PIN_DIR_R, HIGH); DELAY(1500); digitalWrite(PIN_DIR_R, LOW); DELAY(200); }
    RE_BRAKE();
    DI_CHUYEN(v, v);
    DIR_L = 0; DIR_R = 1;
}

void TANG_TOC_TRAI(){ dac_out_L +=5; }
void GIAM_TOC_TRAI(){ dac_out_L -=5; }
void TANG_TOC_PHAI(){ dac_out_R +=5; }
void GIAM_TOC_PHAI(){ dac_out_R -=5; }
void HUMAN_MODE(){ HM_MODE = 1; }
void ROBOT_MODE(){ HM_MODE = 0; }

void DELAY(unsigned long wait_time){
    unsigned long start = millis();
    while (millis() - start < wait_time) {
        DATA_ROS2(); 
    }
}

void DATA_ROS2() {
    static unsigned long last_ros_print = 0;
    if (millis() - last_ros_print >= 50) {
        long pulse_L = 0, pulse_R = 0;
        
        noInterrupts();
        pulse_L = total_pulses_L;
        pulse_R = total_pulses_R;
        interrupts();
        
        Serial.print(pulse_L);
        Serial.print(",");
        Serial.println(pulse_R);
        
        last_ros_print = millis();
    }
}