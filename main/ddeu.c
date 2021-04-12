// Some boiler-plate code was derived from the micro-ROS examples

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uxr/client/config.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_uros/options.h>
#include <sensor_msgs/msg/joint_state.h>
#include <uros_network_interfaces.h>

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__,            \
             (int)temp_rc);                                                    \
      vTaskDelete(NULL);                                                       \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__,          \
             (int)temp_rc);                                                    \
    }                                                                          \
  }

rcl_publisher_t publisher;
sensor_msgs__msg__JointState joint_state_msg;
static double x = 1.23;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &joint_state_msg, NULL));
    x = x + 1.1;
  }
}

void micro_ros_task(void *arg) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t *rmw_options =
      rcl_init_options_get_rmw_init_options(&init_options);
  // Static Agent IP
  RCCHECK(rmw_uros_options_set_udp_address(
      CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator));
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "ddeu_robot", "", &support));
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "joint_states"));

  // create timer,
  rcl_timer_t timer;
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initial Joint State dummy message
  sensor_msgs__msg__JointState__init(&joint_state_msg);
  joint_state_msg.position.data = &x;
  joint_state_msg.position.size = 1;
  joint_state_msg.position.capacity = 1;

  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    usleep(10000);
  }

  // cleanup
  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);
}

void app_main(void) {
#ifdef UCLIENT_PROFILE_UDP
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

  // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
  xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
              CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}