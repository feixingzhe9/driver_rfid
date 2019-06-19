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
    protocol.can_msg.Data[1] = type;
    protocol.can_msg.Data[2] = protocol.serial_num;
    protocol.retry_cnt = 3;
    protocol.expect_len.len_min = 3;
    protocol.expect_len.len_max = 32;
    this->protocol_vector.push_back(protocol);
    ROS_INFO("start to get version . . . ");
    return error;
}

uint8_t DriverRFID::get_dev_id_by_src_id(uint8_t src_id)
{
    if(src_id < RFID_CAN_MAC_SRC_ID_BASE)
    {
        return 0xff;
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
    //if(this->is_log_on == true)
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
        uint8_t len;
        len = msg->Data[1];
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_READ_VERSION");
        protocol_ack.serial_num = msg->Data[0];
        
        std::string version;
        version.clear();
        for (uint8_t i = 0; i < len; i++)
        {
            version.push_back(*(char *)&(msg->Data[i + 2]));
        }
        ROS_INFO("version :%s", version.c_str());
        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->protocol_ack_vector.push_back(protocol_ack);
        }while(0);
#if 0
        get_version_ack_t get_version_ack;
        get_version_ack.get_version_type = msg->Data[0];
        if(get_version_ack.get_version_type == 1)//software version
        {
            sys_powerboard->sw_version.resize(len);
            sys_powerboard->sw_version.clear();
            for(uint8_t i = 0; i < len; i++)
            {
                sys_powerboard->sw_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.sw_version.push_back(*(char *)&(msg->Data[i+2]));
            }

            n.setParam(software_version_param,sys_powerboard->sw_version.data());
        }
        else if(get_version_ack.get_version_type == 2)//protocol version
        {
            sys_powerboard->protocol_version.resize(len);
            sys_powerboard->protocol_version.clear();
            for(uint8_t i = 0; i < len; i++)
            {
                sys_powerboard->protocol_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.protocol_version.push_back(*(char *)&(msg->Data[i+2]));
            }
            n.setParam(protocol_version_param,sys_powerboard->protocol_version.data());
        }
        else if(get_version_ack.get_version_type == 3)//hardware version
        {
            sys_powerboard->hw_version.resize(len);
            sys_powerboard->hw_version.clear();
            //ROS_ERROR("hardware version length: %d",len);
            for(uint8_t i = 0; i < len; i++)
            {
                sys_powerboard->hw_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.hw_version.push_back(*(char *)&(msg->Data[i+2]));
            }
            n.setParam(hardware_version_param,sys_powerboard->hw_version.data());
        }

        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->get_version_ack_vector.push_back(get_version_ack);
        }while(0);

#endif
    }

}

