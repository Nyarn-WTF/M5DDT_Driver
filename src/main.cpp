#include <Arduino.h>
#include <M5Stack.h>
#include "motor_drive.h"
#include "m5_logo.h"
#include "Patlite.h"
#include "ros2_node.h"

motor_drive *motor;
Patlite *patlite;

float dt = 0.01;
float Kp = 20.0;
float Ki = 1.0;
float Kd = 0.2;

int16_t pid_calc(int16_t befor_value, int16_t feedback_rpm, int16_t target_rpm, int16_t error[2]);
void motor_control(void * pvParameters);

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
    motor = new motor_drive(OPEN_LOOP, 0x02);
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

    xTaskCreate(motor_control,  "motor control", 8192, nullptr, 1, nullptr);
}

void loop() {
    M5.update();
    delay(1);
}

void motor_control(void * pvParameters){
    int16_t left_velo = 0, right_velo = 0, left_velo_ave = 0, right_velo_ave = 0;
    int8_t average_count = 0;
    int16_t left_err[2] = {0}, right_err[2] = {0};
    
    while(true){
        if(!motor->get_timeout()){
            feedback_t left_feedback, right_feedback;

            left_feedback = motor->get_left_wheel_feedback();
            right_feedback = motor->get_right_wheel_feedback();

            if(average_count >= 10){
                left_velo = pid_calc(left_velo, left_feedback.velocity, 60, left_err);
                right_velo = pid_calc(right_velo, right_feedback.velocity, -60, right_err);
                Serial.printf("%d, %d, %d, %d\r\n", left_feedback.velocity, right_feedback.velocity, left_velo, right_velo);
                average_count = 0;
            }else{
                left_velo_ave = (left_feedback.velocity + left_velo_ave) / 2;
                right_velo_ave = (right_feedback.velocity + right_velo_ave) / 2;
                average_count++;
            }
        }else{
            left_velo = 0;
            right_velo = 0;
            left_velo_ave = 0;
            right_velo_ave = 0;
            average_count = 0;
            left_err[0] = 0;
            left_err[1] = 0;
            right_err[0] = 0;
            right_err[1] = 0;
        }

        motor->set_velocity(left_velo, right_velo);
        motor->drive();

        delay(1);
    }
}

int16_t pid_calc(int16_t befor_value, int16_t feedback_rpm, int16_t target_rpm, int16_t error[2]){
    float p, i, d;
    error[0] = error[1];
    int32_t error_buf = target_rpm - feedback_rpm;

    if(error_buf >  INT16_MAX - 1)
        error_buf = INT16_MAX - 1;
    else if(error_buf < INT16_MIN + 1)
        error_buf = INT16_MIN + 1;
        
    error[1] = (int16_t)error_buf;

    befor_value += (error[1] + error[0]) / 2.0 * dt;

    p = Kp * error[1];
    i = Ki * befor_value;
    d = Kd * (error[1] - error[0])/dt;

    int32_t rtn = (int32_t)(p + i + d);
    if(rtn > INT16_MAX - 1)
        rtn = INT16_MAX - 1;
    else if(rtn < INT16_MIN + 1)
        rtn = INT16_MIN + 1;

    return (int16_t)rtn;
}