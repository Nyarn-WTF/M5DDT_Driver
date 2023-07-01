#include <Arduino.h>
#include <M5Stack.h>
#include "motor_drive.h"
#include "m5_logo.h"
#include "Patlite.h"
#include "ros2_node.h"

motor_drive *motor;
Patlite *patlite;

SET_LOOP_TASK_STACK_SIZE(64000);

void setup() {
    M5.begin();
    M5.Power.begin();
    Serial.begin(9600);
    Serial.println("setup start");

    Serial2.begin(9600, SERIAL_8N1, 16, 17);

    M5.Lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
    delay(500);
    M5.Lcd.setTextColor(BLACK);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(0xFFFF,0x0000);
    M5.Lcd.setCursor(10, 10);
    delay(500);

    Serial.println("Patlite setup");
    patlite = new Patlite();
    Serial.println("Motor setup");
    motor = new motor_drive(VELOCITY_LOOP, 0x80);
    Serial.println("setup finished");

    if(M5.BtnA.isPressed() && M5.BtnB.isPressed() && M5.BtnC.isReleased()){
        //IDセットモード左 ボタンA:ON B:ON C:OFF
        Serial.println("Set left wheel id");
        motor->set_leftwheel_id();
        delay(1000);
        M5.Power.reset();
    }else if(M5.BtnA.isPressed() && M5.BtnB.isReleased() && M5.BtnC.isPressed()){
        //IDセットモード右 ボタンA:ON B:OFF C:ON
        Serial.println("Set right wheel id");
        motor->set_rightwheel_id();
        delay(1000);
        M5.Power.reset();
    }
}

void loop() {
    static int16_t velo = 10;
    motor->set_rpm_velocity(velo, velo);
    motor->drive();
    M5.update();
    delay(10000);
}