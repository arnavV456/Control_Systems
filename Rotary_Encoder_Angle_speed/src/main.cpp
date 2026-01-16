#include <Arduino.h>

// Encoder pins
#define ENCODER_A 32
#define ENCODER_B 33

// Encoder parameters
const int CPR = 600;            // Counts per revolution
const float TWOPI = 6.2831853;

// Shared variables =
volatile long encoderCount = 0;

// Timing variables
unsigned long lastTime = 0;
long lastCount = 0;

// Output variables
float angle_rad = 0.0;
float speed_rad_s = 0.0;

// Interrupt Service Routine
void IRAM_ATTR encoderISR() {
    int b = digitalRead(ENCODER_B);

    if (b == HIGH)
        encoderCount++;
    else
        encoderCount--;
}

void setup() {
    Serial.begin(115200);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A),
                    encoderISR, RISING);

    lastTime = millis();

    Serial.println("ESP32 Rotary Encoder Initialized");
}

void loop() {
    unsigned long currentTime = millis();
    float Ts = (currentTime - lastTime) / 1000.0;

    if (Ts >= 0.01) {   // 10 ms sampling time

        long currentCount = encoderCount;
        long delta = currentCount - lastCount;

        // ================= Position =================
        // Raw angle in degrees
        float angle_deg = (currentCount * 360.0) / CPR;

        // making angle from 0 to 360
        angle_deg = fmod(angle_deg, 360.0);
        if (angle_deg < 0)
            angle_deg += 360.0;

        // speed
        speed_rad_s = (delta * TWOPI) / (CPR * Ts);

        // output
        Serial.print("Angle (deg): ");
        Serial.print(angle_deg, 2);
        Serial.print(" | Speed (rad/s): ");
        Serial.println(speed_rad_s, 2);

        lastCount = currentCount;
        lastTime = currentTime;
    }
}

