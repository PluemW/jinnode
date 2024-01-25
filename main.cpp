#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include "pin.h"
#include "config.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>

 
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

//------------------------------ < ESP32 Define > -----------------------------------//
static uint32_t preT = 0;

float tick1 = 0.0;
float tick2 = 0.0;
float tick3 = 0.0;
float tick4 = 0.0;
float tick5 = 0.0;
long positions[5] = {yzero,yzero,0,0,0};
// long positions[5] = {0,0,0,0,0};
long pre_positions[5];
int state_lim = 0;
String state_grip = "Idle";
String state_retry = "Idle";
bool isSetZero[6] = {false,false,false,false,false,false};
bool lock_com = false;

AccelStepper st1(1,stpwm1,stdir1);
AccelStepper st2(1,stpwm2,stdir2);
AccelStepper st3(1,stpwm3,stdir3);
AccelStepper st4(1,stpwm4,stdir4);
AccelStepper st5(1,stpwm5,stdir5);
MultiStepper st;

//------------------------------ < Fuction Topic > -----------------------------------//

int lim_switch(int lim_pin);
void task_arduino_fcn(void *arg);
void task_ros_fcn(void *arg);

//------------------------------ < Ros Define > -------------------------------------//
// ? basic
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rcl_allocator_t allocator;
rclc_executor_t executor;

// ? define msg
geometry_msgs__msg__Twist position_msg;
geometry_msgs__msg__Twist step_msg;
std_msgs__msg__String gripper_msg;
std_msgs__msg__String retry_msg;
std_msgs__msg__Bool lock_msg;

// ? define publisher
rcl_publisher_t pub_step;

// ? define subscriber
rcl_subscription_t sub_position;
rcl_subscription_t sub_lock;
rcl_subscription_t sub_statebucket;
rcl_subscription_t sub_stateretry;

rcl_init_options_t init_options;

bool micro_ros_init_successful;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//------------------------------ < Publisher Fuction > ------------------------------//

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    step_msg.linear.x = tick1;
    step_msg.linear.y = tick2;
    step_msg.linear.z = tick3;
    step_msg.angular.x = tick4;
    step_msg.angular.y = tick5;
    step_msg.angular.z = state_lim;
    rcl_publish(&pub_step, &step_msg, NULL);
  }
}

//------------------------------ < Subscriber Fuction > -----------------------------//

void sub_position_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *position_msg = (const geometry_msgs__msg__Twist *)msgin;
  if(position_msg->linear.x != 0.0){
    positions[0] = position_msg->linear.x;
    positions[1] = position_msg->linear.x;
  }
  if(position_msg->linear.y != 0.0){
    positions[2] = position_msg->linear.y;
  }
  if(position_msg->linear.z != 0.0){
    positions[3] = position_msg->linear.z;
  }
  if(position_msg->angular.x != 0.0){
    positions[4] = position_msg->angular.x;
  }
  tick1 = st1.currentPosition();
  tick2 = st2.currentPosition();
  tick3 = st3.currentPosition();
  tick4 = st4.currentPosition();
  tick5 = st5.currentPosition();
}

void sub_lock_callback(const void *msgin)
{
  const std_msgs__msg__Bool *lock_msg = (const std_msgs__msg__Bool *)msgin;
  if(lock_msg->data!=lock_com){
    lock_com = lock_msg->data;
  }
}

void sub_statebucket_callback(const void *msgin)
{
  const std_msgs__msg__String *gripper_msg = (const std_msgs__msg__String *)msgin;
  if(gripper_msg->data.data!="Idle"){
    state_grip = gripper_msg->data.data;
  }
}

void sub_stateretry_callback(const void *msgin)
{
  const std_msgs__msg__String *retry_msg = (const std_msgs__msg__String *)msgin;
  if(retry_msg->data.data!="Idle"){
    state_retry = retry_msg->data.data;
  }
}

//------------------------------ < Ros Fuction > ------------------------------------//

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

  // TODO: create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // TODO: create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_step,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "step/motor"));

  // TODO: create subscriber
  RCCHECK(rclc_subscription_init_default(
      &sub_position,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "gripper/bucket/pos"));
  RCCHECK(rclc_subscription_init_default(
      &sub_lock,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "gripper/bucket/lock"));
  RCCHECK(rclc_subscription_init_default(
      &sub_statebucket,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "state/main"));
  RCCHECK(rclc_subscription_init_default(
      &sub_stateretry,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "state/retry"));

  // TODO: create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_position, &position_msg, &sub_position_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_lock, &lock_msg, &sub_lock_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_statebucket, &gripper_msg, &sub_statebucket_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_stateretry, &retry_msg, &sub_stateretry_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub_step, &node);
  rcl_subscription_fini(&sub_position, &node);
  rcl_subscription_fini(&sub_lock, &node);
  rcl_subscription_fini(&sub_statebucket, &node);
  rcl_subscription_fini(&sub_stateretry, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  xTaskCreatePinnedToCore(
      task_arduino_fcn, /* Task function. */
      "Arduino Task",   /* String with name of task. */
      65536,             /* Stack size in bytes. */
      NULL,             /* Parameter passed as input of the task */
      0,                /* Priority of the task. */
      NULL,0);            /* Task handle. */

  xTaskCreatePinnedToCore(
      task_ros_fcn, /* Task function. */
      "Ros Task",   /* String with name of task. */
      65536,         /* Stack size in bytes. */
      NULL,         /* Parameter passed as input of the task */
      1,            /* Priority of the task. */
      NULL,1);        /* Task handle. */
}

void loop(){
}

//--------------------------------<funcion operation>--------------------------------//

int lim_switch(int lim_pin)
{
  return not digitalRead(lim_pin);
}

void set_zero(){
  if(!isSetZero[5]){
      if (lim_switch(lim1) == HIGH && !isSetZero[0])
      {
          st1.setSpeed(0);
          st1.setCurrentPosition(0);
          positions[0] = 0;
          isSetZero[0] = true;
      }
      if (lim_switch(lim2) == HIGH && !isSetZero[1])
      {
          st2.setSpeed(0);
          st2.setCurrentPosition(0);
          positions[1] = 0;
          isSetZero[1] = true;
      }
      if (lim_switch(lim3) == HIGH && !isSetZero[2])
      {
          st3.setSpeed(0);
          st3.setCurrentPosition(0);
          positions[2] = 0;
          isSetZero[2] = true;
      }
      // if(lim_switch(lim4) == HIGH && !isSetZero[3]){
      //     st4.setCurrentPosition(0);
      //     st4.setSpeed(0);
      //     positions[3] = 0;
      //     isSetZero[3] = true;
      // }
      if (lim_switch(lim5) == HIGH && !isSetZero[4])
      {
          st5.setCurrentPosition(0);
          st5.setSpeed(0);
          positions[4] = 0;
          isSetZero[4] = true;
      }
      if (isSetZero[0] && isSetZero[1] && !isSetZero[2] && !isSetZero[4])
      {
          positions[2] = zzero;
          // positions[3] = zzero;
          positions[4] = zzero;
      }
      if(isSetZero[2] && isSetZero[4] && state_lim==0){
          positions[0] = 18000;  // 18000 , 18000+75000 ,  18000+145000 // 18000+10000
          positions[1] = 18000;
          positions[2] = -15000;  // -25500  
          positions[3] = -15000;  // -33000
          positions[4] = -15000;  // -25500
          isSetZero[5] = true;
          state_lim = 1;
      }
  }
}

void task_arduino_fcn(void *arg)
{
  st1.setMaxSpeed(BMotor_speed); 
  st2.setMaxSpeed(BMotor_speed);
  st3.setMaxSpeed(SMotor_speed);
  st4.setMaxSpeed(MMotor_speed);
  st5.setMaxSpeed(SMotor_speed);  
  st1.setAcceleration(Baccel);
  st2.setAcceleration(Baccel);
  st3.setAcceleration(Saccel);
  st4.setAcceleration(Maccel);
  st5.setAcceleration(Saccel);
  st.addStepper(st1);  
  st.addStepper(st2);
  st.addStepper(st3);
  st.addStepper(st4);
  st.addStepper(st5);
  pinMode(lock,OUTPUT);
  pinMode(lim1,INPUT_PULLUP);
  pinMode(lim2,INPUT_PULLUP);
  pinMode(lim3,INPUT_PULLUP);
  // pinMode(lim4,INPUT_PULLUP);
  pinMode(lim5,INPUT_PULLUP);
  while (true)
  {
    set_zero();
    if ((pre_positions[0] != positions[0]) || (pre_positions[1] != positions[1]) || (pre_positions[2] != positions[2]) || (pre_positions[3] != positions[3]) || (pre_positions[4] != positions[4]))
    {
      pre_positions[0] = positions[0];
      pre_positions[1] = positions[1];
      pre_positions[2] = positions[2];
      pre_positions[3] = positions[3];
      pre_positions[4] = positions[4];
      st.moveTo(positions);
    }
    st.run();
    if(state_grip=="Reset"||state_retry=="First"){
      long positions[5] = {yzero,yzero,0,0,0};
      bool isSetZero[6] = {false,false,false,false,false,false};
      state_lim = 0;
    }
    digitalWrite(lock,lock_com);
  } 
}

void task_ros_fcn(void *arg)
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  while (true)
  {
   switch (state)
   { 
   case WAITING_AGENT:
     EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
     break;
   case AGENT_AVAILABLE:
     state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
     if (state == WAITING_AGENT)
   {
      destroy_entities();
     };
     break;
   case AGENT_CONNECTED:
     EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
     if (state == AGENT_CONNECTED)
     {
       rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
     }
     break;
   case AGENT_DISCONNECTED:
     destroy_entities();
     state = WAITING_AGENT;
     break;
   default:
     break;
   }
  }
}