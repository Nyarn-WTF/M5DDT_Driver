#include <M5Stack.h>
#include "motor_drive.h"
#include <mcp_can.h>

#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

motor_drive::motor_drive(drive_mode_e mode, uint8_t feedback_polling) {
    if(feedback_polling > 128) {
        while(true)
            delay(1000);
    }

    drive_mode = mode;

    //CAN 終端抵抗(0: OFF/ 1: ON)
    uint8_t resistor_stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    CAN0.sendMsgBuf(0x109, 0, 8, resistor_stmp);
    delay(10);

    //モータ フィードバックモード(0x00: Voltage / 0x01: Current / 0x02: Velocity / 0x03: Angle / 0x09: Disabled / 0x0A: Enabled)
    uint8_t mode_stmp[8] = {(uint8_t)drive_mode, (uint8_t)drive_mode, 0, 0, 0, 0, 0, 0};
    CAN0.sendMsgBuf(0x105, 0, 8, mode_stmp);
    delay(10);

    //フィードバック間隔(1~127ms)
    uint8_t feedback_stmp[8] = {feedback_polling, feedback_polling, 0, 0, 0, 0, 0, 0};
    CAN0.sendMsgBuf(0x106, 0, 8, feedback_stmp);
    delay(10);

    xTaskCreate(
        [](void *this_pointer){
            static_cast<motor_drive *>(this_pointer)->feedback_task();
        },
        "Feedback task",
        4000,
        this,
        1,
        nullptr
    );
}

[[noreturn]] void motor_drive::feedback_task() {
    while(true){
        byte left_wheel_byte[8] = {0}, right_wheel_byte[8] = {0};
        byte size = 8;

        CAN0.readMsgBuf(&this->left_motor_id, &size, left_wheel_byte);
        CAN0.readMsgBuf(&this->right_motor_id, &size, right_wheel_byte);

        parse_feedback(&this->left_wheel_feedback, left_wheel_byte);
        parse_feedback(&this->right_wheel_feedback, right_wheel_byte);

        delay(10);
    }
}

void motor_drive::parse_feedback(feedback_t *feedback, const byte data[8]){

    feedback->velocity = (data[0] << 8) & (data[1] & 0x00FF);
    feedback->current = (data[2] << 8) & (data[3] & 0x00FF);
    feedback->angle = (data[4] << 8) & (data[5] & 0x00FF);
    feedback->velocity = data[6];
    feedback->velocity = data[7];

}

bool motor_drive::set_velocity(int16_t velo_left, int16_t velo_right){
    if(drive_mode != OPEN_LOOP)
        return false;

    cmd_stmp[0] = (int8_t)(velo_right >> 8);
    cmd_stmp[1] = (int8_t)(velo_right & 0x00FF);

    cmd_stmp[2] = (int8_t)(velo_left >> 8);
    cmd_stmp[3] = (int8_t)(velo_left & 0x00FF);

    return true;
}

bool motor_drive::set_rpm_velocity(int16_t velo_left, int16_t velo_right){
    if(drive_mode != VELOCITY_LOOP)
        return false;

    if(velo_left < -210 || velo_left > 210)
        return false;

    if(velo_right < -210 || velo_right > 210)
        return false;

    cmd_stmp[0] = (int8_t)(velo_right >> 8);
    cmd_stmp[1] = (int8_t)(velo_right & 0x00FF);

    cmd_stmp[2] = (int8_t)(velo_left >> 8);
    cmd_stmp[3] = (int8_t)(velo_left & 0x00FF);

    return true;
}

bool motor_drive::set_current(int16_t velo_left, int16_t velo_right){
    if(drive_mode != CURRENT_LOOP)
        return false;

    cmd_stmp[0] = (int8_t)(velo_right >> 8);
    cmd_stmp[1] = (int8_t)(velo_right & 0x00FF);

    cmd_stmp[2] = (int8_t)(velo_left >> 8);
    cmd_stmp[3] = (int8_t)(velo_left & 0x00FF);

    return true;
}

bool motor_drive::set_angle(int16_t velo_left, int16_t velo_right){
    if(drive_mode != ANGLE_LOOP)
        return false;

    if(velo_left < 0)
        return false;

    if(velo_right < 0)
        return false;

    cmd_stmp[0] = (int8_t)(velo_right >> 8);
    cmd_stmp[1] = (int8_t)(velo_right & 0x00FF);

    cmd_stmp[2] = (int8_t)(velo_left >> 8);
    cmd_stmp[3] = (int8_t)(velo_left & 0x00FF);

    return true;
}

feedback_t motor_drive::get_right_wheel_feedback(){
    feedback_t feedback = this->right_wheel_feedback;
    return feedback;
}

feedback_t motor_drive::get_left_wheel_feedback(){
    feedback_t feedback = this->left_wheel_feedback;
    return feedback;
}

bool motor_drive::drive(){
    byte res = CAN0.sendMsgBuf(0x32, 0, 8, cmd_stmp);
    delay(10);

    return true;
}