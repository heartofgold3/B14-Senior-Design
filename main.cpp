/**
*
* Created by MacKenna Bochnak 12/9/2024.
* Midterm Corrected main.cpp: Obtain calibrated proximity measurements from APDS-9960 sensor
*
**/
#include "mbed.h"
#include "USBSerial.h"
#include "MemoryPool.h"

// Serial communication for debugging
USBSerial serial;

DigitalOut sensor_power(p22);
DigitalOut i2c_pullup(P1_0);

// I2C configuration
I2C i2c_bus(I2C_SDA1, I2C_SCL1);  
const int sensor_address = 0x39 << 1;  // APDS-9960 7-bit address

// LED configuration
PwmOut blueLED(P0_6);   // PWM pin for blue LED
DigitalOut greenLED(P0_16);  // Digital pin for green LED

// APDS-9960 Registers
const char ENABLE_REG = 0x80;
const char PDATA_REG = 0x9C;
const char CONTROL_REG = 0x8F;

// Threads and Mailbox
Thread proximity_thread;
Thread led_control_thread;

typedef struct {
    uint8_t proximity;
} mail_t;

Mail<mail_t, 16> mailbox;

// Calibration constants
const uint8_t midpoint = 5;          // Adjusted midpoint of proximity range
const uint8_t max_proximity = 10;    // Maximum proximity range in cm

// Enable sensor
void enable_sensor() {
    sensor_power = 1;
    i2c_pullup = 1;
    ThisThread::sleep_for(100ms);

    // Enable proximity detection
    char enable_data[2] = {ENABLE_REG, 0x05};  // PON + PEN
    i2c_bus.write(sensor_address, enable_data, 2);
}

// Check sensor ID
bool check_sensor_id() {
    char id_data[1] = {0x92};
    char id_value[1];
    i2c_bus.write(sensor_address, id_data, 1);
    i2c_bus.read(sensor_address, id_value, 1);
    return id_value[0] == 0xAB;
}

// Calculate distance
int calculate_distance(uint8_t proximity) {
    int distance_mm = 0; // Distance in millimeters

    if (proximity >= 170) {
        // Segment 1: 0–1 cm (0–10 mm)
        distance_mm = 0 + ((proximity - 170) * 1000) / (255 - 170);
    } else if (proximity >= 70) {
        // Segment 2: 1–2 cm (10–20 mm)
        distance_mm = 1000 + ((proximity - 70) * 1000) / (170 - 70);
    } else if (proximity >= 41) {
        // Segment 3: 2–3 cm (20–30 mm)
        distance_mm = 2000 + ((proximity - 41) * 1000) / (70 - 41);
    } else if (proximity >= 26) {
        // Segment 4: 3–4 cm (30–40 mm)
        distance_mm = 3000 + ((proximity - 26) * 1000) / (41 - 26);
    } else if (proximity >= 14) {
        // Segment 5: 4–5 cm (40–50 mm)
        distance_mm = 4000 + ((proximity - 14) * 1000) / (26 - 14);
    } else if (proximity >= 11) {
        // Segment 6: 5–6 cm (50–60 mm)
        distance_mm = 5000 + ((proximity - 11) * 1000) / (14 - 11);
    } else if (proximity >= 8) {
        // Segment 7: 6–7 cm (60–70 mm)
        distance_mm = 6000 + ((proximity - 8) * 1000) / (11 - 8);
    } else if (proximity >= 6) {
        // Segment 8: 7–8 cm (70–80 mm)
        distance_mm = 7000 + ((proximity - 6) * 1000) / (8 - 6);
    } else if (proximity >= 5) {
        // Segment 9: 8–9 cm (80–90 mm)
        distance_mm = 8000 + ((proximity - 5) * 1000) / (6 - 5);
    } else if (proximity >= 4) {
        // Segment 10: 9–10 cm (90–100 mm)
        distance_mm = 9000 + ((proximity - 4) * 1000) / (5 - 4);
    } else {
        // Out of range
        distance_mm = 10000; // Max measurable distance (10 cm)
    }

    return distance_mm / 1000; // Convert millimeters to centimeters
}


// Proximity thread function
void proximity() {
    if (!check_sensor_id()) {
        serial.printf("Sensor ID mismatch.\r\n");
        return;
    }

    char read_proximity[1] = {PDATA_REG};
    char proximity_data[1];

    while (true) {
        if (i2c_bus.write(sensor_address, read_proximity, 1) == 0 &&
            i2c_bus.read(sensor_address, proximity_data, 1) == 0) {
            uint8_t proximity = proximity_data[0];

            // Adjusted distance calculation using integer values
            int distance = calculate_distance(proximity);
            serial.printf("Proximity: %d | Distance: %d cm\r\n", proximity, distance);

            mail_t *mail = mailbox.alloc();
            mail->proximity = proximity;
            mailbox.put(mail);
        }
        ThisThread::sleep_for(100ms);
    }
}


// LED control thread
void led_control() {
    while (true) {
        osEvent evt = mailbox.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t *)evt.value.p;
            uint8_t proximity = mail->proximity;

            if (proximity < max_proximity) {
                float intensity = 1.0f - (float(proximity) / max_proximity);
                blueLED.write(intensity);
                greenLED = 0;  // Turn off green LED
            } else {
                blueLED.write(0.0f);
                greenLED = 1;  // Turn on green LED
            }
            mailbox.free(mail);
        }
    }
}

int main() {
    serial.printf("Starting proximity sensor program\r\n");

    enable_sensor();

    proximity_thread.start(proximity);
    led_control_thread.start(led_control);

    while (true) {
        ThisThread::sleep_for(1000ms);
    }
}