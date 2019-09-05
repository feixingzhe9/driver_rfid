#include <signal.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <driver_rfid.h>
#include <can_protocol.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_rfid");
    ROS_INFO("start to create driver_rfid node . . .");

    DriverRFID *driver_rfid = new DriverRFID();

    pthread_t can_protocol_proc_handle;
    pthread_create(&can_protocol_proc_handle, NULL, can_protocol_handle,(void*)driver_rfid);


    float rate = 1000;
    ros::Rate loop_rate(rate);

    bool init_flag = false;
    sleep(2);
    while(ros::ok())
    {
        if(!init_flag)
        {
            driver_rfid->get_auth_rfid_version(1);
            driver_rfid->get_auth_rfid_version(2);
            driver_rfid->get_auth_rfid_version(3);
            driver_rfid->get_dst_src_info_rfid_version(1);
            driver_rfid->get_dst_src_info_rfid_version(2);
            driver_rfid->get_dst_src_info_rfid_version(3);
            driver_rfid->get_cabinet_detection_rfid_version(1);
            driver_rfid->get_cabinet_detection_rfid_version(2);
            driver_rfid->get_cabinet_detection_rfid_version(3);
            init_flag = true;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
