#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <geometry_msgs/msg/twist.h>

void updateDisplay(const geometry_msgs__msg__Twist* msg_sub);

#endif
