#include <signal.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <driver_rfid.h>
#include <can_protocol.h>


void sigintHandler(int sig)
{
    ROS_INFO("killing on exit");
    ros::shutdown();
}

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
    sleep(3);
    while(ros::ok())
    {
        if(!init_flag)
        {
            for(uint8_t i = 0; i < RFID_MAX_NUM; i++)
            {
                driver_rfid->get_version(i, 1);
            }
            init_flag = true;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}