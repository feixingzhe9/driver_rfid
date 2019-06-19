#include <driver_rfid.h>

void *can_protocol_handle(void *pdata)
{
    DriverRFID *driver_rfid = (DriverRFID *)pdata;

    protocol_t protocol;
    protocol_ack_t protocol_ack;
    bool start_protocol_flag = false;
    bool get_protocol_ack_flag = false;

    while(ros::ok())
    {
        do
        {

            boost::mutex::scoped_lock(mtx);
            if(!driver_rfid->protocol_vector.empty())
            {
                auto a = driver_rfid->protocol_vector.begin();
                protocol = *a;
                driver_rfid->protocol_vector.erase(a);
                start_protocol_flag = true;
            }
        }while(0);

        if(start_protocol_flag)
        {
            start_protocol_flag = false;
            CAN_ID_UNION can_id_send;
            CAN_ID_UNION can_id_ack;
            driver_rfid->pub_msg_to_can(protocol.can_msg);
            ROS_INFO("send can msg");

            uint8_t retry_cnt = protocol.retry_cnt;
            can_id_send.CANx_ID = protocol.can_msg.ID;
            protocol_ack_t protocol_ack;
            while(retry_cnt > 0)
            {
                usleep(100 * 1000);
                ROS_INFO("cnt : %d", retry_cnt);
                retry_cnt--;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if (!driver_rfid->protocol_ack_vector.empty())
                    {
                        auto b = driver_rfid->protocol_ack_vector.begin();
                        protocol_ack = *b;
                        driver_rfid->protocol_ack_vector.erase(b);
                        get_protocol_ack_flag = true;
                    }
                }while (0);

                if(get_protocol_ack_flag)
                {
                    get_protocol_ack_flag = false;
                    can_id_ack.CANx_ID = protocol_ack.can_msg.ID;
                    ROS_INFO("get ack of %d", can_id_ack.CanID_Struct.SourceID);
                    if(can_id_send.CanID_Struct.SourceID == can_id_ack.CanID_Struct.SourceID)
                    {
                        if ((protocol_ack.can_msg.DataLen >= protocol.expect_len.len_min) && (protocol_ack.can_msg.DataLen <= protocol.expect_len.len_max))
                        {
                            if(protocol_ack.serial_num == protocol.serial_num)
                            {

                            }
                        }
                    }
                }
            }
        }
        usleep(10 * 1000);

    }
}