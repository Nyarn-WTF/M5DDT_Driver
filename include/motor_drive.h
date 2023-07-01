#ifndef M5DDT_DRIVER_MOTOR_DRIVE_H
#define M5DDT_DRIVER_MOTOR_DRIVE_H

typedef enum {
    OPEN_LOOP = 0x00,
    CURRENT_LOOP = 0x01,
    VELOCITY_LOOP = 0x02,
    ANGLE_LOOP = 0x03,
    DISABLED_ = 0x09,
    ENABLED_ = 0x0A
} drive_mode_e;

typedef enum {
    NO_FAULT = 0x00,
    UNDER_VOLTAGE1 = 0x01,
    UNDER_VOLTAGE2 = 0x02,
    OVER_VOLTAGE = 0x03,
    OVER_CURRENT = 0x0A,
    OVER_VELOCITY = 0x14,
    OVER_TEMPERATURE1 = 0x1E,
    OVER_TEMPERATURE2 = 0x1F,
    ANGLE_SENSOR_FAILURE = 0x2A,
    ABNORMAL_ANGLE_SENSOR_SIGNAL = 0x2B,
    UNABLE_TO_ACCESS_THE_MOTOR = 0x3C,
    PHASE_LOSS_OF_THREE_PHASE_LINE = 0x51
} fault_value_e;

typedef struct {
    int16_t velocity;
    int16_t current;
    int16_t angle;
    fault_value_e fault_value;
    drive_mode_e mode;
}feedback_t;

class motor_drive {
private:
    drive_mode_e drive_mode = OPEN_LOOP;
    unsigned long left_motor_id = 0x02 + 0x96;
    unsigned long right_motor_id = 0x01 + 0x96;

    feedback_t right_wheel_feedback = {0}, left_wheel_feedback = {0};

    uint8_t cmd_stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    [[noreturn]] void feedback_task();
    static void parse_feedback(feedback_t *feedback, const byte data[8]);

public:
    motor_drive(drive_mode_e mode, uint8_t feedback_polling);
    bool set_velocity(int16_t velo_left, int16_t velo_right);
    bool set_rpm_velocity(int16_t velo_left, int16_t velo_right);
    bool set_current(int16_t velo_left, int16_t velo_right);
    bool set_angle(int16_t velo_left, int16_t velo_right);
    void set_rightwheel_id();
    void set_leftwheel_id();
    feedback_t get_right_wheel_feedback();
    feedback_t get_left_wheel_feedback();
    bool drive();
};


#endif //M5DDT_DRIVER_MOTOR_DRIVE_H
