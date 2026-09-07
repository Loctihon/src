#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            Serial.println("ESP32: Tay cầm đã kết nối!");
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Serial.println("ESP32: Mất kết nối tay cầm!");
            break;
        }
    }
}

void setup() {
    Serial.begin(115200);       // Dùng để debug với máy tính
    Serial2.begin(9600, SERIAL_8N1, 16, 17); // UART kết nối sang Arduino Mega (TX2=17, RX2=16)

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    Serial.println("ESP32 đang đợi kết nối tay cầm...");
}

void loop() {
    BP32.update();

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr myController = myControllers[i];

        if (myController && myController->isConnected()) {
            uint8_t dpad = myController->dpad();

            // 4.1. Test 2 nút cò LB/LT
            if (myController->l1()) { 
                Serial2.write('Q');  
                delay(100); 
            }
            else if (myController->l2()){
                Serial2.write('A'); 
                delay(100); 
            }

            // TEST 2 NÚT RB/RT
            if (myController->r1()) { 
                Serial2.write('E'); 
                delay(100); 
            }
            else if (myController->r2()){
                Serial2.write('D'); 
                delay(100); 
            }
            
            // Đọc phím điều hướng và gửi sang Arduino Mega
            if (dpad == 0x01) {
                Serial2.write('F'); // F = Forward (Tiến)
                delay(100);
            }
            else if (dpad == 0x02) {
                Serial2.write('B'); // B = Backward (Lùi)
                delay(100);
            }
            else if (dpad == 0x08) {
                Serial2.write('L'); // L = Left (Trái)
                delay(100);
            }
            else if (dpad == 0x04) {
                Serial2.write('R'); // R = Right (Phải)
                delay(100);
            }
            else if (myController->b()) {
                Serial2.write('S'); // S = Stop / Brake (Phanh)
                delay(100);
            }
            else if (myController->x()) {
                Serial2.write('X'); 
                delay(100);
            }
            else if (myController->y()) {
                Serial2.write('Y'); 
                delay(100);
            }
            else if (myController->a()) {
                Serial2.write('T'); 
                delay(100);
            }
        }
    }
    delay(20);
}