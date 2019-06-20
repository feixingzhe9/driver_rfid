/*
 *  Author: Kaka Xie
 *  brief: CAN protocol
 */

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
            if(can_id_send.CanID_Struct.ACK == 0)
            {
                while (retry_cnt > 0)
                {
                    uint32_t timeout_cnt = 50;
                    while (timeout_cnt > 0)
                    {
                        usleep(10 * 1000);
                        timeout_cnt -= 1;
                        do
                        {
                            boost::mutex::scoped_lock(mtx);
                            if (!driver_rfid->protocol_ack_vector.empty())
                            {
                                auto b = driver_rfid->protocol_ack_vector.begin();
                                protocol_ack = *b;
                                driver_rfid->protocol_ack_vector.erase(b);
                                get_protocol_ack_flag = true;
                                timeout_cnt = 0;
                            }
                        } while (0);
                    }

                    if (get_protocol_ack_flag)
                    {
                        get_protocol_ack_flag = false;
                        can_id_ack.CANx_ID = protocol_ack.can_msg.ID;
                        ROS_INFO("get ack, source id: 0x%x,  src_mac_id: 0x%x", can_id_ack.CanID_Struct.SourceID, can_id_ack.CanID_Struct.SrcMACID);
                        if (can_id_send.CanID_Struct.SourceID == can_id_ack.CanID_Struct.SourceID)
                        {
                            ROS_INFO("get right source id: 0x%x ", can_id_send.CanID_Struct.SourceID);
                            if ((protocol_ack.can_msg.DataLen >= protocol.expect_len.len_min) && (protocol_ack.can_msg.DataLen <= protocol.expect_len.len_max))
                            {
                                ROS_INFO("get right expect data len: %d ", protocol_ack.can_msg.DataLen);
                                if (protocol_ack.serial_num == protocol.serial_num)
                                {
                                    ROS_INFO("protocol get right ack");
                                    break;
                                }
                                else
                                {
                                    ROS_ERROR("send serial num: %d,  ack serial num: %d", protocol.serial_num, protocol_ack.serial_num);
                                }
                            }
                            else
                            {
                                ROS_ERROR("ack CAN data len: %d,  expect data len MIN: %d,  expect data len MAX: %d",
                                          protocol_ack.can_msg.DataLen, protocol.expect_len.len_min, protocol.expect_len.len_max);
                            }
                        }
                        else
                        {
                            ROS_ERROR("ack source id: 0x%x,  send source id: 0x%x", can_id_ack.CanID_Struct.SourceID, can_id_send.CanID_Struct.SourceID);
                        }
                    }
                    else //wait for ack timeout
                    {
                        driver_rfid->pub_msg_to_can(protocol.can_msg);
                        ROS_ERROR("time out resend can msg");
                        retry_cnt--;
                    }
                }
            }
        }
        usleep(10 * 1000);
    }
}