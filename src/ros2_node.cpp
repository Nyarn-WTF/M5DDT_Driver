//
// Created by wn on 6/10/23.
//

#include <Arduino.h>
#include <M5Stack.h>
#include "ros2_node.h"


ros2_node::ros2_node(String node_name, String node_namespace, String feedback_publisher_topic_name, String cmd_vel_subscriber_topic_name, String patlite_subscriber_topic_name){
    //Create Node
    {
        allocator = rcl_get_default_allocator();
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        rcl_init_options_init(&init_options, allocator);
        rcl_init_options_set_domain_id(&init_options, 10);
        rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        rcl_ret_t rc = rclc_node_init_default(&node, node_name.c_str(), node_namespace.c_str(), &support);
        if (rc != RCL_RET_OK) {
            while(true);
        }
    }

    //Create feedback publisher
    {
        const rosidl_message_type_support_t *type_support =
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
        rcl_ret_t rc = rclc_publisher_init_default(
                &feedback_publisher, &node,
                type_support, feedback_publisher_topic_name.c_str());
        if (RCL_RET_OK != rc) {
            while (true);
        }
    }

    //Create cmd_vel subscriber
    {
        const rosidl_message_type_support_t *type_support =
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
        rcl_ret_t rc = rclc_subscription_init_default(
                &cmd_vel_subscriber, &node,
                type_support, cmd_vel_subscriber_topic_name.c_str());
        rc = rclc_executor_add_subscription(
                &executor, &cmd_vel_subscriber, &cmd_vel_msg,
                cmd_vel_callback, ON_NEW_DATA);
        if (RCL_RET_OK != rc) {
            while (true);
        }
    }

    //Create patlite subscriber
    {
        const rosidl_message_type_support_t *type_support =
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
        rcl_ret_t rc = rclc_subscription_init_default(
                &patlite_subscriber, &node,
                type_support, patlite_subscriber_topic_name.c_str());
        rc = rclc_executor_add_subscription(
                &executor, &patlite_subscriber, &patlite_msg,
                patlite_callback, ON_NEW_DATA);
        if (RCL_RET_OK != rc) {
            while (true);
        }
    }
}

void ros2_node::cmd_vel_callback(const void * msgin){

}

void ros2_node::patlite_callback(const void * msgin){

}

ros2_node::~ros2_node() {
    rcl_publisher_fini(&feedback_publisher, &node);
    rcl_subscription_fini(&cmd_vel_subscriber, &node);
    rcl_subscription_fini(&patlite_subscriber, &node);
    rcl_node_fini(&node);
}