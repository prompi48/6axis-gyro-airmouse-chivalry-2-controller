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
// UUID และพิน
// ==========================================
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef01-1234-5678-1234-56789abcdef0"

#define LEFTBUTTON 26
#define RIGHTBUTTON 25
#define JOY_X 33
#define JOY_Y 32
#define JOY_SW 27   // ยังไม่ได้ใช้งาน

#define MOUSE_LEFT 1
#define MOUSE_RIGHT 2
#define MOUSE_MIDDLE 4
#define MOUSE_BACK 8
#define MOUSE_FORWARD 16

// ==========================================
// ค่าคงที่ (ปรับจูนได้ที่นี่ที่เดียว)
// ==========================================
constexpr unsigned long MOVE_INTERVAL_MS   = 10;    // ส่งการขยับเมาส์ ~100Hz
constexpr unsigned long DEBOUNCE_MS        = 20;    // กันปุ่มเด้ง
constexpr unsigned long MODE_HOLD_MS       = 2500;  // กดซ้าย+ขวาค้างเพื่อสลับโหมด
constexpr unsigned long MPU_WAKE_DELAY_MS  = 3000;  // รอหลังเชื่อมต่อก่อนปลุก MPU
constexpr unsigned long MPU_SETTLE_MS      = 500;   // รอ MPU นิ่งหลังปลุก
constexpr unsigned long COOLDOWN_MS        = 200;   // หน่วงระหว่างท่าทาง
constexpr float GYRO_DEADZONE              = 0.05f;
constexpr float SPEED_SCALE                = 100.0f; // ตัวคูณความเร็ว (ต่อวินาที)
constexpr float GESTURE_CONFIDENCE         = 0.7f;
constexpr int   JOY_DEAD_LOW               = 1700;
constexpr int   JOY_DEAD_HIGH              = 2300;

#define FREQUENCY_HZ 50
#define INTERVAL_MS (1000 / FREQUENCY_HZ)

// ==========================================
// ตัวแปรสถานะ
// ==========================================
int currentSpeed = 10;
uint8_t currentLeftMapCode = 1;
uint8_t currentRightMapCode = 2;
int currentJoySpeed = 12;

Adafruit_MPU6050 mpu;
bool sleepMPU = true;
unsigned long mpuReadyAt = 0;
bool wasConnected = false;
unsigned long connectedAt = 0;

volatile bool isChivMode = false;   // แก้จาก callback BLE และ loop
bool deviceConnected = false;

unsigned long lastMoveTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastActionTime = 0;
float accX = 0, accY = 0;           // เก็บเศษทศนิยมของการขยับ ไม่ให้สูญหาย

// การสลับโหมดและคอมโบปุ่ม
bool comboActive = false;
bool modeSwitched = false;
unsigned long comboStart = 0;

// Edge Impulse
float feature_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t buffer_index = 0;

BLEHIDDevice* hid;
BLECharacteristic* inputMouse;
uint8_t _buttons = 0;

// ==========================================
// ปุ่มกดพร้อม debounce
// ==========================================
struct Button {
    uint8_t pin;
    bool stable;
    bool lastRaw;
    unsigned long changedAt;
    Button(uint8_t p) : pin(p), stable(false), lastRaw(false), changedAt(0) {}
    void update() {
        bool raw = !digitalRead(pin);
        unsigned long now = millis();
        if (raw != lastRaw) { lastRaw = raw; changedAt = now; }
        if (now - changedAt >= DEBOUNCE_MS) stable = raw;
    }
};
Button btnL(LEFTBUTTON);
Button btnR(RIGHTBUTTON);
bool leftHeld = false;
bool rightHeld = false;

// ==========================================
// BLE callbacks
// ==========================================
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
    USAGE(2),           0x38, 0x02,
    LOGICAL_MINIMUM(1), 0x81,
    LOGICAL_MAXIMUM(1), 0x7f,
    REPORT_SIZE(1),     0x08,
    REPORT_COUNT(1),    0x01,
    HIDINPUT(1),        0x06,
    END_COLLECTION(0),
    END_COLLECTION(0)
};

// ==========================================
// ฟังก์ชันเมาส์
// ==========================================
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, feature_buffer + offset, length * sizeof(float));
    return 0;
}

uint8_t getMouseAction(uint8_t code) {
    switch (code) {
        case 1: return MOUSE_LEFT;
        case 2: return MOUSE_RIGHT;
        case 3: return MOUSE_MIDDLE;
        case 4: return MOUSE_FORWARD;
        case 5: return MOUSE_BACK;
        default: return MOUSE_LEFT;
    }
}

void sendReport(uint8_t buttons, int8_t x, int8_t y, int8_t wheel = 0, int8_t hWheel = 0) {
    if (!deviceConnected) return;
    uint8_t m[5] = { buttons, (uint8_t)x, (uint8_t)y, (uint8_t)wheel, (uint8_t)hWheel };
    inputMouse->setValue(m, 5);
    inputMouse->notify();
}

// รับ int แล้ว clamp เป็น -127..127 กัน overflow
void mouseMove(int x, int y, int wheel = 0, int hWheel = 0) {
    sendReport(_buttons,
               (int8_t)constrain(x, -127, 127),
               (int8_t)constrain(y, -127, 127),
               (int8_t)constrain(wheel, -127, 127),
               (int8_t)constrain(hWheel, -127, 127));
}

// คลิกแบบไม่ทับปุ่มที่กดค้างอยู่
void mouseClick(uint8_t b) {
    sendReport(_buttons | b, 0, 0);
    sendReport(_buttons, 0, 0);
}

void mousePress(uint8_t b)   { _buttons |= b;  sendReport(_buttons, 0, 0); }
void mouseRelease(uint8_t b) { _buttons &= ~b; sendReport(_buttons, 0, 0); }

void applyButton(bool want, bool &held, uint8_t mask) {
    if (want && !held)       { held = true;  mousePress(mask); }
    else if (!want && held)  { held = false; mouseRelease(mask); }
}

void releaseAllButtons() {
    _buttons = 0;
    leftHeld = rightHeld = false;
    sendReport(0, 0, 0);
}

void resetMotionState() {
    accX = accY = 0;
    buffer_index = 0;
}

// แปลงค่า ADC ของจอยเป็น -speed..+speed (นอก deadzone เท่านั้น)
int joyAxis(int raw) {
    if (raw > JOY_DEAD_HIGH) return map(raw, JOY_DEAD_HIGH, 4095, 1, currentJoySpeed);
    if (raw < JOY_DEAD_LOW)  return -map(raw, JOY_DEAD_LOW, 0, 1, currentJoySpeed);
    return 0;
}

// ==========================================
// รับค่าตั้งค่าจากเว็บ
// ==========================================
class WebSettingsCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string rx = pCharacteristic->getValue();
        if (rx.length() < 5) return;

        int newSpeed = (uint8_t)rx[0];
        if (newSpeed >= 1 && newSpeed <= 50) currentSpeed = newSpeed;

        uint8_t l = (uint8_t)rx[1];
        uint8_t r = (uint8_t)rx[2];
        if (l >= 1 && l <= 5) currentLeftMapCode = l;
        if (r >= 1 && r <= 5) currentRightMapCode = r;

        int newJoy = (uint8_t)rx[3];
        if (newJoy >= 1 && newJoy <= 30) currentJoySpeed = newJoy;

        if (rx.length() >= 6) isChivMode = (rx[5] != 0);

        Serial.println("====== รับค่าตั้งค่าใหม่จากเว็บ ======");
        Serial.printf("Air speed: %d | Joy speed: %d | ChivMode: %s\n",
                      currentSpeed, currentJoySpeed, isChivMode ? "ON" : "OFF");
    }
};

// ==========================================
// จำแนกท่าทางด้วย Edge Impulse
// ==========================================
void runGestureClassifier() {
    ei_signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &raw_feature_get_data;

    ei_impulse_result_t result = {0};
    if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

    size_t best = 0;
    float maxValue = 0;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > maxValue) {
            maxValue = result.classification[i].value;
            best = i;
        }
    }

    if (maxValue <= GESTURE_CONFIDENCE || millis() - lastActionTime <= COOLDOWN_MS) return;

    const char* label = result.classification[best].label;
    bool fired = true;
    if      (strcmp(label, "slash2left") == 0)    mouseClick(MOUSE_LEFT);
    else if (strcmp(label, "slash2right") == 0)   mouseClick(MOUSE_FORWARD);
    else if (strcmp(label, "overhead2left") == 0) mouseMove(0, 0, -1);
    else if (strcmp(label, "overhead2right") == 0) mouseClick(MOUSE_BACK);
    else if (strcmp(label, "stab") == 0)          mouseMove(0, 0, 1);
    else fired = false;

    if (fired) lastActionTime = millis();
}

// ==========================================
// setup
// ==========================================
void setup() {
    Serial.begin(115200);

    pinMode(LEFTBUTTON, INPUT_PULLUP);
    pinMode(RIGHTBUTTON, INPUT_PULLUP);
    pinMode(JOY_SW, INPUT_PULLUP);

    BLEDevice::init("Chivalry_AirMouse");

    // ตั้งค่าความปลอดภัยก่อนสร้าง service
    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    hid = new BLEHIDDevice(pServer);
    inputMouse = hid->inputReport(0);

    hid->manufacturer()->setValue("MyESP32");
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->hidInfo(0x00, 0x02);
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

// ==========================================
// loop (ไม่มี delay ที่บล็อกการทำงาน)
// ==========================================
void loop() {
    unsigned long now = millis();

    // ---------- ยังไม่เชื่อมต่อ ----------
    if (!deviceConnected) {
        if (wasConnected) {              // เพิ่งหลุด: เคลียร์สถานะ
            wasConnected = false;
            if (!sleepMPU) { mpu.enableSleep(true); sleepMPU = true; }
            _buttons = 0;
            leftHeld = rightHeld = false;
            comboActive = modeSwitched = false;
            resetMotionState();
        }
        delay(20);                        // ยังไม่เชื่อมต่อ ไม่มีอะไรต้องรีบทำ
        return;
    }

    if (!wasConnected) { wasConnected = true; connectedAt = now; }

    // ---------- ปลุก MPU แบบไม่บล็อก ----------
    if (sleepMPU) {
        if (now - connectedAt < MPU_WAKE_DELAY_MS) return;
        mpu.enableSleep(false);
        sleepMPU = false;
        mpuReadyAt = now + MPU_SETTLE_MS;
        Serial.println("MPU6050 awakened!");
        return;
    }
    if ((long)(now - mpuReadyAt) < 0) return;

    // ---------- ปุ่ม + สลับโหมด ----------
    btnL.update();
    btnR.update();

    if (btnL.stable && btnR.stable) {
        if (!comboActive) {
            comboActive = true;
            comboStart = now;
        } else if (!modeSwitched && now - comboStart >= MODE_HOLD_MS) {
            isChivMode = !isChivMode;
            modeSwitched = true;          // ต้องปล่อยปุ่มทั้งคู่ก่อนจึงสลับได้อีก
            releaseAllButtons();
            resetMotionState();
            Serial.print("MODE SWITCHED! Chivalry 2 Mode: ");
            Serial.println(isChivMode ? "ON (Combat)" : "OFF (Air Mouse)");
        }
    } else if (!btnL.stable && !btnR.stable) {
        comboActive = false;
        modeSwitched = false;
    }

    // กดค้าง/ปล่อยแบบเมาส์จริง (ลากได้) และไม่ทำงานระหว่างคอมโบสลับโหมด
    bool wantL = btnL.stable && !comboActive;
    bool wantR = btnR.stable && !comboActive;
    applyButton(wantL, leftHeld,  getMouseAction(currentLeftMapCode));
    applyButton(wantR, rightHeld, getMouseAction(currentRightMapCode));

    // ---------- โหมด Air Mouse ----------
    if (!isChivMode) {
        if (now - lastMoveTime >= MOVE_INTERVAL_MS) {
            float dt = (now - lastMoveTime) / 1000.0f;
            lastMoveTime = now;

            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            float gyroZ = g.gyro.z;
            float gyroY = g.gyro.y;
            if (fabsf(gyroZ) < GYRO_DEADZONE) gyroZ = 0;
            if (fabsf(gyroY) < GYRO_DEADZONE) gyroY = 0;

            // สะสมค่า เพื่อไม่ให้การขยับช้าๆ หายไปตอนปัดเศษ
            accX += gyroZ * currentSpeed * SPEED_SCALE * dt;
            accY -= gyroY * currentSpeed * SPEED_SCALE * dt;

            int dx = constrain((int)accX, -127, 127);
            int dy = constrain((int)accY, -127, 127);
            if (dx != 0 || dy != 0) {
                accX -= dx;
                accY -= dy;
                mouseMove(dx, dy);
            }
        }
        return;
    }

    // ---------- โหมด Chivalry ----------
    // จอยสติ๊ก (ส่งเฉพาะตอนมีการขยับ)
    if (now - lastMoveTime >= MOVE_INTERVAL_MS) {
        lastMoveTime = now;
        int dx = -joyAxis(analogRead(JOY_Y));
        int dy =  joyAxis(analogRead(JOY_X));
        if (dx != 0 || dy != 0) mouseMove(dx, dy);
    }

    // เก็บข้อมูลเซ็นเซอร์ 50Hz สำหรับ AI
    if (now - lastSampleTime >= INTERVAL_MS) {
        lastSampleTime = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        if (buffer_index + 6 <= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            feature_buffer[buffer_index++] = a.acceleration.x;
            feature_buffer[buffer_index++] = a.acceleration.y;
            feature_buffer[buffer_index++] = a.acceleration.z;
            feature_buffer[buffer_index++] = g.gyro.x;
            feature_buffer[buffer_index++] = g.gyro.y;
            feature_buffer[buffer_index++] = g.gyro.z;
        }

        if (buffer_index + 6 > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            runGestureClassifier();
            buffer_index = 0;
        }
    }
}
