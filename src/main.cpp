#include <M5Stack.h>
#include "motor_drive.h"
#include "m5_logo.h"

motor_drive *motor;

void setup() {
    M5.begin();
    M5.Power.begin();
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);

    M5.Lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
    delay(500);
    M5.Lcd.setTextColor(BLACK);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(0xFFFF,0x0000);
    M5.Lcd.setCursor(10, 10);
    delay(500);

    motor = new motor_drive(VELOCITY_LOOP, 0x80);
}

void loop() {
    static int16_t velo = 0;
    static bool toggle = false;
    motor->set_rpm_velocity(velo, velo);
    motor->drive();
    M5.update();

    if(!toggle) {
        velo++;
        if(velo > 20)
            toggle = true;

    }else{
        velo--;
        if(velo < -20)
            toggle = false;
    }
    delay(100);
}