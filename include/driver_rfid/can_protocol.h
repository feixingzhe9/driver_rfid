#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <boost/thread/mutex.hpp>
#include "std_msgs/UInt8MultiArray.h"


void *can_protocol_handle(void *pdata);
