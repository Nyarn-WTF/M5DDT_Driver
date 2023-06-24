//
// Created by wn on 6/10/23.
//

#ifndef M5DDT_DRIVER_ROS2_NODE_H
#define M5DDT_DRIVER_ROS2_NODE_H

#include <string>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

class ros2_node {
private:
    rcl_publisher_t feedback_publisher;
    rcl_subscription_t cmd_vel_subscriber, patlite_subscriber;

    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rcl_timer_t timer;

    std_msgs__msg__Int32 cmd_vel_msg, patlite_msg;

    void cmd_vel_callback(const void * msgin);
    void patlite_callback(const void * msgin);
public:
    ros2_node(String node_name, String node_namespace, String feedback_publisher_topic_name, String cmd_vel_subscriber_topic_name, String patlite_subscriber_topic_name);
    ~ros2_node();
};


#endif //M5DDT_DRIVER_ROS2_NODE_H
