/*
 *  Author: Kaka Xie
 *  brief: rfid
 */

#include <driver_rfid.h>

void DriverRFID::pub_msg_to_can(mrobot_msgs::vci_can can_msg)
{
    this->pub_to_can.publish(can_msg);
}

int DriverRFID::get_version(uint8_t dev_id, uint8_t type)
{
    int error = 0;
    CAN_ID_UNION id;
    protocol_t protocol;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_VERSION;
    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = RFID_CAN_MAC_SRC_ID_BASE + dev_id;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    protocol.serial_num = this->protocol_serial_num++;
    protocol.can_msg.ID = id.CANx_ID;
    protocol.can_msg.DataLen = 3;
    protocol.can_msg.Data.resize(3);
    protocol.can_msg.Data[0] = 0;
    protocol.can_msg.Data[1] = protocol.serial_num;
    protocol.can_msg.Data[2] = type;
    protocol.retry_cnt = 3;
    protocol.expect_len.len_min = 3;
    protocol.expect_len.len_max = 32;
    this->protocol_vector.push_back(protocol);
    ROS_INFO("start to get version . . . ");
    return error;
}


int DriverRFID::ack_mcu_upload(uint8_t dev_id, CAN_ID_UNION id, uint8_t serial_num)
{
    ROS_INFO("start to ack mcu upload info. . . ");
    int error = 0;
    mrobot_msgs::vci_can can_msg;

    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = RFID_CAN_MAC_SRC_ID_BASE + dev_id;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 1;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = serial_num;
    this->pub_msg_to_can(can_msg);
    return error;
}

void DriverRFID::pub_rfid_info(uint8_t dev_id, uint16_t data)
{
    std_msgs::UInt16MultiArray info;
    info.data.clear();
    info.data.push_back(dev_id);
    info.data.push_back(data);
    this->rfid_pub.publish(info);
}

uint8_t DriverRFID::get_dev_id_by_src_id(uint8_t src_id)
{
    if(src_id < RFID_CAN_MAC_SRC_ID_BASE)
    {
        return 0xff;
    }
    if(src_id == RFID_CAN_MAC_SRC_ID_BASE + 0x0f - 1)
    {
        return 0x0f - 1;
    }
    if(src_id > RFID_CAN_MAC_SRC_ID_BASE + RFID_MAX_NUM)
    {
        return 0xff;
    }
    return src_id - RFID_CAN_MAC_SRC_ID_BASE;
}

void DriverRFID::rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg)
{
    mrobot_msgs::vci_can can_msg;
    mrobot_msgs::vci_can long_msg;
    CAN_ID_UNION id;

    long_msg = this->long_frame.frame_construct(c_msg);
    mrobot_msgs::vci_can* msg = &long_msg;
    if( msg->ID == 0 )
    {
        return;
    }

    if(0)
    {
        for(uint8_t i = 0; i < msg->DataLen; i++)
        {
            ROS_INFO("msg->Data[%d] = 0x%x",i,msg->Data[i]);
        }
    }
    can_msg.ID = msg->ID;
    id.CANx_ID = can_msg.ID;
    can_msg.DataLen = msg->DataLen;
    uint8_t data_len = msg->DataLen;

    uint8_t dev_id = this->get_dev_id_by_src_id(id.CanID_Struct.SrcMACID);
    if(dev_id == 0xff)
    {
        ROS_ERROR("get wrong src id : %d", id.CanID_Struct.SourceID);
    }
    ROS_INFO("get dev id: %d", dev_id);
    protocol_ack_t protocol_ack;
    protocol_ack.can_msg = *msg;

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_READ_VERSION)
    {
        protocol_ack.serial_num = msg->Data[0];
        uint8_t version_type = msg->Data[1];
        uint8_t len = msg->Data[2];
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_READ_VERSION");

        switch(version_type)
        {
            case 1:
                for (uint8_t i = 0; i < len; i++)
                {
                    this->sw_version[dev_id].push_back(*(char *)&(msg->Data[i + 3]));
                }
                ROS_INFO("MCU software version :%s", this->sw_version[dev_id].c_str());
                break;

            case 2:
                for (uint8_t i = 0; i < len; i++)
                {
                    this->protocol_version[dev_id].push_back(*(char *)&(msg->Data[i + 3]));
                }
                ROS_INFO("MCU protocol_version version :%s", this->protocol_version[dev_id].c_str());
                break;

            case 3:
                for (uint8_t i = 0; i < len; i++)
                {
                    this->hw_version[dev_id].push_back(*(char *)&(msg->Data[i + 3]));
                }
                ROS_INFO("MCU hardware version :%s", this->hw_version[dev_id].c_str());
                break;
        }

        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->protocol_ack_vector.push_back(protocol_ack);
        } while (0);
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_RFID_INFO)
    {
        if (data_len == 5)
        {
            std_msgs::String rfid;
            uint8_t status = 1;
            int id_type = 0;
            uint16_t rfid_int = 0;
            rfid.data.resize(4);
            rfid.data.clear();
            rfid_int = /*(msg->Data[0] << 24) | (msg->Data[1] << 16) |*/ (msg->Data[2] << 8) | msg->Data[3];
            for(uint8_t i = 0; i < 4; i++)
            {
                ROS_INFO("msg->Data[%d]: 0x%x", i, msg->Data[i]);
            }

            ROS_INFO("rfid int data :%d", rfid_int);

            CAN_ID_UNION id;
            uint8_t serial_num = msg->Data[msg->DataLen - 1];
            id.CanID_Struct.ACK = 1;
            id.CanID_Struct.SourceID = CAN_SOURCE_ID_RFID_INFO;
            this->ack_mcu_upload(dev_id, id, serial_num);
            this->pub_rfid_info(dev_id, rfid_int);
        }
    }


}

