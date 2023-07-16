#include <Arduino.h>
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
double wheel_tread = 0.40;

int16_t pid_calc(int16_t befor_value, int16_t feedback_rpm, int16_t target_rpm, int16_t error[2]);
void motor_control(void * pvParameters);
void ros2_setup();

SET_LOOP_TASK_STACK_SIZE(64000);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#define M5STACK_FIRE_NEO_NUM_LEDS 10
#define M5STACK_FIRE_NEO_DATA_PIN 15

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(M5STACK_FIRE_NEO_NUM_LEDS, M5STACK_FIRE_NEO_DATA_PIN, NEO_GRB + NEO_KHZ800);

void error_loop(){
  while(1){
    int r = 255;
    int g = 0;
    int b = 0;
    for(int i = 0; i < 9; i++){
        pixels.setPixelColor(i, pixels.Color(r, g, b));   
    }  
    pixels.show();
    delay(100);
  }
}

void setup() {
    M5.begin();
    M5.Power.begin();
    Serial.begin(2000000);
    Serial.println("setup start");

    int r = 0;
    int g = 0;
    int b = 0;
    for(int i = 0; i < 9; i++){
        pixels.setPixelColor(i, pixels.Color(r, g, b));   
    }  
    pixels.show();

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

    xTaskCreatePinnedToCore(motor_control,  "motor control", 32000, nullptr, 1, nullptr, 0);
}

void loop() {

    M5.update();

    if(!motor->get_timeout()){
        double vr = (double)motor->get_right_wheel_feedback().velocity / 60 * wheel_phi * PI;
        double vl = (double)motor->get_left_wheel_feedback().velocity / 60 * wheel_phi * PI;
        double x = (vr + vl) / 2;
        double z = (vr - vl) / (2 * wheel_tread);
        odom_msg.linear.x = x;
        odom_msg.angular.z = z;
        RCSOFTCHECK(rcl_publish(&publisher, &odom_msg, NULL));
        M5.Lcd.clear(BLACK);
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.printf("Twixt\nLX: %lf\nAZ: %lf\n", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
        M5.Lcd.printf("Odom \nLX: %lf\nAZ: %lf\n", x, z);
    }
    RCCHECK(rclc_executor_spin_one_period(&executor, 0));
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
                float vl = cmd_vel_msg.linear.x - pow(wheel_tread, -1) * cmd_vel_msg.angular.z;
                float vr = 2 * cmd_vel_msg.linear.x - vl;
                int16_t target_vl = vl * 60 / wheel_phi / PI;
                int16_t target_vr = vr * 60 / wheel_phi / PI;

                left_velo = pid_calc(left_velo, left_velo_ave, target_vl, left_err);
                right_velo = pid_calc(right_velo, right_velo_ave, target_vr, right_err);
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