#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include <Adafruit_MPU6050.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// ==========================================
// กำหนด UUID และพินต่างๆ
// ==========================================
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef01-1234-5678-1234-56789abcdef0"

#define LEFTBUTTON 26
#define RIGHTBUTTON 25
#define JOY_X 33 
#define JOY_Y 32 
#define JOY_SW 27 

#define MOUSE_LEFT 1
#define MOUSE_RIGHT 2
#define MOUSE_MIDDLE 4
#define MOUSE_BACK 8
#define MOUSE_FORWARD 16

int currentSpeed = 10; 
uint8_t currentLeftMapCode = 1;  
uint8_t currentRightMapCode = 2; 
int currentJoySpeed = 12; 
//uint8_t currentJoySwMapCode = 3; 

Adafruit_MPU6050 mpu;
bool sleepMPU = true;
bool isChivMode = false; 

unsigned long bothPressedMillis = 0;
bool isBothPressed = false;
unsigned long lastActionTime = 0;
const unsigned long COOLDOWN_MS = 200; 

#define FREQUENCY_HZ 50
#define INTERVAL_MS (1000 / FREQUENCY_HZ)
unsigned long lastSampleTime = 0;
float feature_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
int buffer_index = 0;

BLEHIDDevice* hid;
BLECharacteristic* inputMouse;
bool deviceConnected = false;
uint8_t _buttons = 0;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println(">>> BLE Client Connected <<<");
    }
    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println(">>> BLE Client Disconnected <<<");
        pServer->startAdvertising(); 
    }
};

static const uint8_t _hidReportDescriptor[] = {
    USAGE_PAGE(1),      0x01, 
    USAGE(1),           0x02, 
    COLLECTION(1),      0x01, 
    USAGE(1),           0x01, 
    COLLECTION(1),      0x00, 
    USAGE_PAGE(1),      0x09, 
    USAGE_MINIMUM(1),   0x01, 
    USAGE_MAXIMUM(1),   0x05, 
    LOGICAL_MINIMUM(1), 0x00, 
    LOGICAL_MAXIMUM(1), 0x01, 
    REPORT_SIZE(1),     0x01, 
    REPORT_COUNT(1),    0x05, 
    HIDINPUT(1),        0x02, 
    REPORT_SIZE(1),     0x03, 
    REPORT_COUNT(1),    0x01, 
    HIDINPUT(1),        0x03, 
    USAGE_PAGE(1),      0x01, 
    USAGE(1),           0x30, 
    USAGE(1),           0x31, 
    USAGE(1),           0x38, 
    LOGICAL_MINIMUM(1), 0x81, 
    LOGICAL_MAXIMUM(1), 0x7f, 
    REPORT_SIZE(1),     0x08, 
    REPORT_COUNT(1),    0x03, 
    HIDINPUT(1),        0x06, 
    USAGE_PAGE(1),      0x0c, 
    USAGE(2),     0x38, 0x02, 
    LOGICAL_MINIMUM(1), 0x81, 
    LOGICAL_MAXIMUM(1), 0x7f, 
    REPORT_SIZE(1),     0x08, 
    REPORT_COUNT(1),    0x01, 
    HIDINPUT(1),        0x06, 
    END_COLLECTION(0),        
    END_COLLECTION(0)         
};

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, feature_buffer + offset, length * sizeof(float));
    return 0;
}

uint8_t getMouseAction(uint8_t code) {
    switch(code) {
        case 1: return MOUSE_LEFT;
        case 2: return MOUSE_RIGHT;
        case 3: return MOUSE_MIDDLE;
        case 4: return MOUSE_FORWARD;
        case 5: return MOUSE_BACK;
        default: return MOUSE_LEFT;
    }
}

void mouseMove(signed char x, signed char y, signed char wheel = 0, signed char hWheel = 0) {
    if (deviceConnected) {
        uint8_t m[5];
        m[0] = _buttons;
        m[1] = x;
        m[2] = y;
        m[3] = wheel;
        m[4] = hWheel;
        inputMouse->setValue(m, 5);
        inputMouse->notify();
    }
}

void mouseClick(uint8_t b) {
    _buttons = b;
    mouseMove(0, 0);
    _buttons = 0;
    mouseMove(0, 0);
}

void mousePress(uint8_t b) {
    _buttons |= b;
    mouseMove(0, 0);
}

void mouseRelease(uint8_t b) {
    _buttons &= ~b;
    mouseMove(0, 0);
}

// คลาสรับค่าจากเว็บ (รองรับทั้ง 5 ไบต์ และไบต์ที่ 6 สลับโหมด)
class WebSettingsCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() >= 5) {
            int newSpeed = rxValue[0];
            if(newSpeed >= 1 && newSpeed <= 50) currentSpeed = newSpeed;
            currentLeftMapCode = rxValue[1];
            currentRightMapCode = rxValue[2];
            int newJoySpeed = rxValue[3];
            if(newJoySpeed >= 1 && newJoySpeed <= 30) currentJoySpeed = newJoySpeed;
            //currentJoySwMapCode = rxValue[4];

            if (rxValue.length() >= 6) {
                isChivMode = (rxValue[5] != 0);
            }
            
            Serial.println("====== รับค่าตั้งค่าใหม่จากเว็บ ======");
            Serial.printf("ความเร็ว Air Mouse: %d | ChivMode: %s\n", currentSpeed, isChivMode ? "ON" : "OFF");
            Serial.println("======================================");
        }
    }
};

void setup() {
    Serial.begin(115200);

    pinMode(LEFTBUTTON, INPUT_PULLUP);
    pinMode(RIGHTBUTTON, INPUT_PULLUP);
    pinMode(JOY_SW, INPUT_PULLUP);

    BLEDevice::init("Chivalry_AirMouse");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    hid = new BLEHIDDevice(pServer);
    inputMouse = hid->inputReport(0);

    hid->manufacturer()->setValue("MyESP32");
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->hidInfo(0x00, 0x02);

    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

    hid->reportMap((uint8_t*)_hidReportDescriptor, sizeof(_hidReportDescriptor));
    hid->startServices();

    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCharacteristic->setCallbacks(new WebSettingsCallback());
    pService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_MOUSE);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    delay(1000);
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 Found!");
    mpu.enableSleep(sleepMPU);
}

void loop() {
    if (deviceConnected) {
        if (sleepMPU) {
            delay(3000);
            Serial.println("MPU6050 awakened!");
            sleepMPU = false;
            mpu.enableSleep(sleepMPU);
            delay(500);
        }

        bool leftState = !digitalRead(LEFTBUTTON);
        bool rightState = !digitalRead(RIGHTBUTTON);
        //bool joySwState = !digitalRead(JOY_SW);

        if (leftState && rightState) {
            if (!isBothPressed) {
                isBothPressed = true;
                bothPressedMillis = millis();
            } else {
                if (millis() - bothPressedMillis >= 2500) {
                    isChivMode = !isChivMode;
                    Serial.print("MODE SWITCHED! Chivalry 2 Mode: ");
                    Serial.println(isChivMode ? "ON (Combat)" : "OFF (Air Mouse)");
                    delay(1000);
                    isBothPressed = false;
                }
            }
        } else { isBothPressed = false; }

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // --- ระบบกรอง Deadzone (ตัด Noise เล็กๆ น้อยๆ ตอนวางนิ่งๆ ทิ้งไป) ---
        float gyroZ = g.gyro.z;
        float gyroY = g.gyro.y;
        
        // ถ้าค่าน้อยกว่า 0.05 (ปรับแต่งเพิ่มลดได้ตามความเหมาะสม) ให้ถือว่าเป็น 0
        if (abs(gyroZ) < 0.05) gyroZ = 0;
        if (abs(gyroY) < 0.05) gyroY = 0;

        float moveX = gyroZ * currentSpeed;
        float moveY = gyroY * -currentSpeed;

        if (!isChivMode) {
            // ส่งค่าเฉพาะตอนที่มีการขยับจริงๆ หรือป้องกันการส่ง 0 ซ้ำซ้อนพร่ำเพรื่อ
            if (moveX != 0 || moveY != 0) {
                mouseMove(moveX, moveY);
            }

            if (leftState && !isBothPressed) {
                mouseClick(getMouseAction(currentLeftMapCode));
                delay(300); 
            }
            if (rightState && !isBothPressed) {
                mouseClick(getMouseAction(currentRightMapCode));
                delay(300); 
            }
            //if (joySwState) {
            //    mouseClick(getMouseAction(currentJoySwMapCode));
            //    delay(300);
            //}
        } else {
            int joyX = analogRead(JOY_Y);
            int joyY = analogRead(JOY_X);
            int joyMoveX = 0;
            int joyMoveY = 0;

            if (joyX > 2300) { joyMoveX = -map(joyX, 2300, 4095, 1, currentJoySpeed); }
            else if (joyX < 1700) { joyMoveX = -map(joyX, 1700, 0, -1, -currentJoySpeed); }

            if (joyY > 2300) { joyMoveY = -map(joyY, 2300, 4095, -1, -currentJoySpeed); }
            else if (joyY < 1700) { joyMoveY = -map(joyY, 1700, 0, 1, currentJoySpeed); }

            mouseMove(joyMoveX, joyMoveY);

            if (leftState && !isBothPressed) {
                mouseClick(getMouseAction(currentLeftMapCode));
                delay(300);
            }
            if (rightState && !isBothPressed) {
                mousePress(getMouseAction(currentRightMapCode));
            } else {
                mouseRelease(getMouseAction(currentRightMapCode));
            }

            //if (joySwState) {
            //    mouseClick(getMouseAction(currentJoySwMapCode));
            //    delay(300);
            //}

            if (millis() - lastSampleTime >= INTERVAL_MS) {
                lastSampleTime = millis();

                if (buffer_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                    feature_buffer[buffer_index++] = a.acceleration.x;
                    feature_buffer[buffer_index++] = a.acceleration.y;
                    feature_buffer[buffer_index++] = a.acceleration.z;
                    feature_buffer[buffer_index++] = g.gyro.x;
                    feature_buffer[buffer_index++] = g.gyro.y;
                    feature_buffer[buffer_index++] = g.gyro.z;
                }

                if (buffer_index >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                    ei_signal_t signal;
                    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
                    signal.get_data = &raw_feature_get_data;

                    ei_impulse_result_t result = {0};
                    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

                    if (res == EI_IMPULSE_OK) {
                        int predicted_index = 0;
                        float max_value = 0;
                        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                            if (result.classification[i].value > max_value) {
                                max_value = result.classification[i].value;
                                predicted_index = i;
                            }
                        }

                        if (max_value > 0.7 && (millis() - lastActionTime > COOLDOWN_MS)) {
                            String label = result.classification[predicted_index].label;

                            if (label == "slash2left") { mouseClick(MOUSE_LEFT); lastActionTime = millis(); }
                            else if (label == "slash2right") { mouseClick(MOUSE_FORWARD); lastActionTime = millis(); }
                            else if (label == "overhead2left") { mouseMove(0, 0, -1); lastActionTime = millis(); }
                            else if (label == "overhead2right") { mouseClick(MOUSE_BACK); lastActionTime = millis(); }
                            else if (label == "stab") { mouseMove(0, 0, 1); lastActionTime = millis(); }
                        }
                    }
                    buffer_index = 0;
                }
            }
        }
    }
}