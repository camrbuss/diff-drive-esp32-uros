// Some boiler-plate code was derived from the micro-ROS examples

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

#include "driver/mcpwm.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rotary_encoder.h"
#include "soc/mcpwm_periph.h"

#include "uxr/client/config.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/point32.h>
#include <uros_network_interfaces.h>

#include "rosidl_runtime_c/string_functions.h"

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
static double _pos[2] = {0.0, 0.0};
static double _vel[2] = {0.0, 0.0};
const char *_names[2] = {"axis0", "axis1"};
const char *_frame_id = "/ddeu";

rcl_subscription_t subscriber;
geometry_msgs__msg__Point32 vel_point;
// TODO: make these thread safe
float left_vel_setpoint = 0;
float right_vel_setpoint = 0;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  joint_state_msg.header.stamp.sec = ts.tv_sec;
  joint_state_msg.header.stamp.nanosec = ts.tv_nsec;
  RCSOFTCHECK(rcl_publish(&publisher, &joint_state_msg, NULL));
}

void vel_callback(const void *msg) {
  const geometry_msgs__msg__Point32 * point_msg = (const geometry_msgs__msg__Point32*)msg;
  left_vel_setpoint = (float) point_msg->x;
  right_vel_setpoint = (float) point_msg->y;
  ESP_LOGI("Vel from cmd_vel/vels: ", "%f - %f", left_vel_setpoint, right_vel_setpoint);
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

  // executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // create timer,
  rcl_timer_t timer;
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "cmd_vel/vels"));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &vel_point, &vel_callback, ON_NEW_DATA));

  // Initial Joint State message, array length 2 for left and right side
  sensor_msgs__msg__JointState__init(&joint_state_msg);
  joint_state_msg.position.data = _pos;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.capacity = 2;
  joint_state_msg.velocity.data = _vel;
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.capacity = 2;
  // Add names to joint state axis
  rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 2);
  for (uint8_t i = 0; i < 2; i++) {
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[i], _names[i]);
  }
  // Add header to message
  rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, _frame_id);

  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // cleanup
  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);
}

void motor_control_task(void *arg) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 2; // in ticks, TODO: make ms
  xLastWakeTime = xTaskGetTickCount();

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 33);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 27);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, 25);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, 26);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 20000;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

  rotary_encoder_t *encoder_l = NULL;
  rotary_encoder_t *encoder_r = NULL;
  uint32_t pcnt_unit_0 = 0;
  uint32_t pcnt_unit_1 = 1;

  // Create rotary encoder instances
  rotary_encoder_config_t config_l =
      ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_0, 5, 18);
  rotary_encoder_config_t config_r =
      ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_1, 17, 16);
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_l, &encoder_l));
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_r, &encoder_r));

  // Filter out glitch (1us)
  ESP_ERROR_CHECK(encoder_l->set_glitch_filter(encoder_l, 1));
  ESP_ERROR_CHECK(encoder_r->set_glitch_filter(encoder_r, 1));

  ESP_ERROR_CHECK(encoder_l->start(encoder_l));
  ESP_ERROR_CHECK(encoder_r->start(encoder_r));
  while (1) {
    // TODO: add thread safe mechanism for non-Atomic Double write
    _pos[0] = encoder_l->get_counter_value(encoder_l);
    _pos[1] = encoder_r->get_counter_value(encoder_r);
    if(left_vel_setpoint < 0.0f) {
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    } else {
      mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    }
    if (right_vel_setpoint < 0.0f) {
      mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
    } else {
      mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, abs(left_vel_setpoint*100.0f));
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, abs(right_vel_setpoint*100.0f));
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void app_main(void) {
#ifdef UCLIENT_PROFILE_UDP
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

  xTaskCreate(motor_control_task, "enc_task", 8000, NULL, 5, NULL);
  // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
  xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
              CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
