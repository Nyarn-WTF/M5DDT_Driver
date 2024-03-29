#include <Arduino.h>
#include "esp_task_wdt.h"
#include <M5Stack.h>
#include <Adafruit_NeoPixel.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include "motor_drive.h"
#include "m5_logo.h"
#include "Patlite.h"

motor_drive *motor;
Patlite *patlite;

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
geometry_msgs__msg__Twist cmd_vel_msg, odom_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

float dt = 0.01;
float Kp = 20.0;
float Ki = 1.0;
float Kd = 0.2;

double wheel_phi = 0.20;
double wheel_tread = 0.20;

int16_t target_vl = 0;
int16_t target_vr = 0;

int16_t pid_calc(int16_t befor_value, int16_t feedback_rpm, int16_t target_rpm, int16_t error[2]);
void motor_control(void * pvParameters);
void ros2_setup();
void ros2_loop(void *pvParameters);

SET_LOOP_TASK_STACK_SIZE(16000);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop_rc();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop_soft();}}

#define M5STACK_FIRE_NEO_NUM_LEDS 10
#define M5STACK_FIRE_NEO_DATA_PIN 15

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(M5STACK_FIRE_NEO_NUM_LEDS, M5STACK_FIRE_NEO_DATA_PIN, NEO_GRB + NEO_KHZ800);

void error_loop_rc(){
  while(1){
    M5.Lcd.clear(BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("RC Error");
    delay(100);
  }
}

void error_loop_soft(){
  while(1){
    M5.Lcd.clear(BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("Soft Error");
    delay(100);
  }
}

void setup() {
    M5.begin();
    M5.Power.begin();
    Serial.begin(2000000);
    Serial.println("setup start");

    delay(1000);

    Serial2.begin(9600, SERIAL_8N1, 16, 17);

    M5.Lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
    delay(500);

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

    ros2_setup();

    for(int i = 0; i < 12; i++){
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        pixels.show();  
    }  

    xTaskCreatePinnedToCore(motor_control,  "motor control", 16000, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(ros2_loop,  "ros2 loop", 16000, nullptr, 2, nullptr, 1);
}

void loop() {

    M5.update();

    if(!motor->get_timeout()){
        M5.Lcd.clear(BLACK);
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.printf("Cmd \nLX: %lf\nAZ: %lf\n", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
        M5.Lcd.printf("Odom \nLX: %lf\nAZ: %lf\n", odom_msg.linear.x, odom_msg.angular.z);
        M5.Lcd.printf("RPM \nRF: %d RT: %d\nLT: %d LR: %d\n", motor->get_left_wheel_feedback().velocity, target_vl, motor->get_right_wheel_feedback().velocity, target_vr);
    }
    delay(1);
}

void motor_control(void * pvParameters){
    int16_t left_velo = 0, right_velo = 0, left_velo_ave = 0, right_velo_ave = 0;
    int8_t average_count = 0;
    int16_t left_err[2] = {0}, right_err[2] = {0};
    
    while(true){
        feedback_t left_feedback, right_feedback;

        if(!motor->get_timeout()){
            left_feedback = motor->get_left_wheel_feedback();
            right_feedback = motor->get_right_wheel_feedback();

            if(average_count >= 10){
                float vl = cmd_vel_msg.linear.x - cmd_vel_msg.angular.z * wheel_tread;
                float vr = cmd_vel_msg.linear.x + cmd_vel_msg.angular.z * wheel_tread;
                target_vl = vl * 60 / wheel_phi / PI;
                target_vr = vr * 60 / wheel_phi / PI;

                left_velo = pid_calc(left_velo, left_velo_ave, target_vl, left_err);
                right_velo = pid_calc(right_velo, right_velo_ave, -target_vr, right_err);
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
            target_vl = 0;
            target_vr = 0;
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

//ROS2

//twist message cb
void subscription_callback(const void *msgin) {
    memcpy(&cmd_vel_msg, (geometry_msgs__msg__Twist*)msgin, sizeof(geometry_msgs__msg__Twist));
}

void ros2_setup(){
    set_microros_serial_transports(Serial);
    
    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "m5stack", "", &support));

    // // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/odom"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));
}

void ros2_loop(void *pvParameters){
    while(1){
        if(!motor->get_timeout()){
            double vr = -(double)motor->get_right_wheel_feedback().velocity / 60 * wheel_phi * PI;
            double vl = (double)motor->get_left_wheel_feedback().velocity / 60 * wheel_phi * PI;
            double x = (vr + vl) / 2;
            double z = (vr - vl) / (2 * wheel_tread);
            odom_msg.linear.x = x;
            odom_msg.angular.z = z;
            rcl_publish(&publisher, &odom_msg, NULL);
        }else{
            odom_msg.linear.x = 0;
            odom_msg.angular.z = 0;
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));
        Serial.flush();
        delay(100);
    }
}