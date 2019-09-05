/*
 *  Author: Kaka Xie
 *  brief: rfid
 */

#include <driver_rfid.h>

using json = nlohmann::json;

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


void DriverRFID::get_cabinet_detection_rfid_version(uint8_t type)
{
#if CABINET_DETECT_RFID_NUM > 0
    for(uint8_t i = CABINET_DETECT_RFID_ID_START; i <= CABINET_DETEDT_RFID_ID_END; i++)
    {
        this->get_version(i, type);
    }
#endif
}

void DriverRFID::get_dst_src_info_rfid_version(uint8_t type)
{
#if DST_SRC_INFO_RFID_NUM > 0
    for(uint8_t i = DST_SRC_INFO_RFID_ID_START; i <= DST_SRC_INFO_RFID_ID_END; i++)
    {
        this->get_version(i, type);
    }
#endif
}


void DriverRFID::get_auth_rfid_version(uint8_t type)
{
#if AUTH_RFID_NUM == 1
    this->get_version(0x0f, type);
#endif
#if AUTH_RFID_NUM == 2
    this->get_version(0x0f, type);
    this->get_version(0, type);
#endif
#if AUTH_RFID_NUM == 3
    this->get_version(0x0f, type);
    this->get_version(0, type);
    this->get_version(0x0e, type);
#endif
}

int DriverRFID::ack_mcu_upload(uint8_t dev_id, CAN_ID_UNION id, uint8_t serial_num)
{
    ROS_INFO("start to ack mcu upload info. . . ");
    ROS_INFO("ack upload info serial num: %d ", serial_num);
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

void DriverRFID::pub_rfid_info(uint8_t dev_id, uint8_t type, uint16_t data)
{
    std_msgs::UInt16MultiArray info;
    info.data.clear();
    if(type == 0)
    {
        info.data.push_back(dev_id);
        info.data.push_back(data);
    }
    else if(type == 1)
    {
        info.data.push_back(dev_id);
    }
    else
    {
        ROS_ERROR("%s: type error: %d", __func__, type);
    }
    this->rfid_pub.publish(info);
}

uint8_t DriverRFID::get_dev_id_by_src_id(uint8_t src_id)
{
    if(src_id < RFID_CAN_MAC_SRC_ID_BASE)
    {
        return 0xff;    //parameter error
    }
    if(src_id > RFID_CAN_MAC_SRC_ID_BASE + RFID_MAX_NUM)
    {
        return 0xff;    //parameter error
    }
    return src_id - RFID_CAN_MAC_SRC_ID_BASE;
}


uint8_t DriverRFID::get_rfid_type_by_dev_id(uint8_t dev_id)
{
    if((dev_id >= CABINET_DETECT_RFID_ID_START) && (dev_id <= CABINET_DETEDT_RFID_ID_END))
    {
        return RFID_TYPE_CABINET_DETECT;
    }

    if((dev_id >= DST_SRC_INFO_RFID_ID_START) && (dev_id <= DST_SRC_INFO_RFID_ID_END))
    {
        return RFID_TYPE_DST_SRC_INFO;
    }

    //    if(((dev_id >= AUTH_RFID_ID_START) && (dev_id <= 16))  || (dev_id == 0))
    {
        return RFID_TYPE_AUTH;
    }
    //    return RFID_TYPE_NONE;
}




uint8_t DriverRFID::get_rfid_type_index(uint8_t dev_id)
{
    if(dev_id < 16)
    {
        if((dev_id >= CABINET_DETECT_RFID_ID_START) && (dev_id <= CABINET_DETEDT_RFID_ID_END))
        {
            return dev_id - CABINET_DETECT_RFID_ID_START;
        }

        else if((dev_id >= DST_SRC_INFO_RFID_ID_START) && (dev_id <= DST_SRC_INFO_RFID_ID_END))
        {
            return dev_id - DST_SRC_INFO_RFID_ID_START;
        }

        else if(dev_id == 0)
        {
            return 3;
        }
        return dev_id - AUTH_RFID_ID_START;
    }
    return 0xff;
}

std::string DriverRFID::hex_2_str(uint8_t hex)
{
    std::string str;
    char c[2];
    str.clear();
    sprintf(c, "%02x", hex);
    str.push_back(c[0]);
    str.push_back(c[1]);
    return str;
}

void DriverRFID::old_pub_rfid_info(std::string uid, std::string type, std::string data)
{
    nlohmann::json j;
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    j.clear();
    j =
        {
            {"pub_name", "rfid_info"},
            {"msg", "success" },
            {"error_code", "0" },
            {
                "data",
                {
                    {"key", uid.c_str()},
                    {"type", type.c_str()},
                    {"data", data.c_str()},
                }
            }
         };

    ss.clear();
    ss << j;
    pub_json_msg.data = ss.str();
    this->old_rfid_pub.publish(pub_json_msg);
}


int DriverRFID::report_rfid_info(uint8_t rfid_type, uint8_t index, uint8_t act, uint32_t rfid_info, uint32_t dst_info, uint32_t src_info)
{
    std::string rfid_type_str;
    std::string act_str;
    switch(act)
    {
        case 0:
            act_str = "depart";
            break;

        case 1:
            act_str = "arrive";
            break;

        default:
            ROS_ERROR("%s: parameter error !  act:%d", __func__, act);
    }
    switch(rfid_type)
    {
        case RFID_TYPE_CABINET_DETECT:
            rfid_type_str = "cabinet_detection";
            break;

        case RFID_TYPE_AUTH:
            rfid_type_str = "auth";
            break;

        case RFID_TYPE_DST_SRC_INFO:
            rfid_type_str = "dst_src_info";
            break;

        default:
            ROS_ERROR("%s: parameter error! rfid_type: %d", __func__, rfid_type);
            return -1;
    }

    json j;
    j.clear();
    j =
    {
        {"rfid_type", rfid_type_str.c_str()},
        {"act", act_str.c_str()},
        {"index", index},
        {"info", rfid_info},
        {"dst_info", dst_info},
        {"src_info", src_info},
    };
    std_msgs::String pub_json_msg;
    std::stringstream ss;
    ss.clear();
    ss << j;
    pub_json_msg.data.clear();
    pub_json_msg.data = ss.str();
    ROS_INFO("%s: pub json string: %s", __func__, pub_json_msg.data.c_str());
    report_rfid_info_pub.publish(pub_json_msg);

}

void DriverRFID::old_rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg)
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

    can_msg.ID = msg->ID;
    id.CANx_ID = can_msg.ID;
    can_msg.DataLen = msg->DataLen;
    uint8_t data_len = msg->DataLen;

    uint8_t source_id = id.CanID_Struct.SourceID;

    switch(source_id)
    {
        case OLD_CAN_SOURCE_ID_RFID_UID:
            ROS_INFO("get source id of rfid uid");
            this->old_rfid_uid.clear();
            for(uint8_t i = 0; i < data_len; i++)
            {
                this->old_rfid_uid += hex_2_str(msg->Data[i]);
            }
            ROS_INFO("uid: %s", this->old_rfid_uid.c_str());
            break;

        case OLD_CAN_SOURCE_ID_RFID_TYPE:
            ROS_INFO("get source id of rfid type");
            this->old_rfid_type.clear();
            for(uint8_t i = 0; i < data_len; i++)
            {
                this->old_rfid_type += hex_2_str(msg->Data[i]);
            }
            ROS_INFO("rfid type: %s", this->old_rfid_type.c_str());
            break;

        case OLD_CAN_SOURCE_ID_RFID_DATA:
            ROS_INFO("get source id of rfid data");
            this->old_rfid_data.clear();
            for(uint8_t i = 0; i < data_len; i++)
            {
                this->old_rfid_data += hex_2_str(msg->Data[i]);
            }
            ROS_INFO("rfid data: %s", this->old_rfid_data.c_str());
            this->old_pub_rfid_info(this->old_rfid_uid, this->old_rfid_type, this->old_rfid_data);
            break;
    }
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
    uint8_t rfid_type = get_rfid_type_by_dev_id(dev_id);
    uint8_t index = get_rfid_type_index(dev_id);
#if 1   //log
    switch(rfid_type)
    {
        case RFID_TYPE_CABINET_DETECT:
            ROS_INFO("get rfid type CABINET_DEETCTION");
            break;

        case RFID_TYPE_DST_SRC_INFO:
            ROS_INFO("get rfid type DST_SRC_INFO");
            break;

        case RFID_TYPE_AUTH:
            ROS_INFO("get rfid type AUTH");
            break;

        default:
            ROS_ERROR("rfid type error !");
            break;
    }
#endif
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
            this->pub_rfid_info(dev_id, 0, rfid_int);
            report_rfid_info(rfid_type, index, 1, rfid_int, 0, 0);
        }
    }



    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_UPLOAD_RFID_DEPART)
    {
        if (data_len == 2)
        {
            ROS_INFO("get CAN_SOURCE_ID_UPLOAD_RFID_DEPART");
            CAN_ID_UNION id;
            uint8_t serial_num = msg->Data[msg->DataLen - 1];
            id.CanID_Struct.ACK = 1;
            id.CanID_Struct.SourceID = CAN_SOURCE_ID_UPLOAD_RFID_DEPART;
            this->ack_mcu_upload(dev_id, id, serial_num);
            this->pub_rfid_info(dev_id, 1, 0);
            report_rfid_info(rfid_type, index, 0, 0, 0, 0);
        }
    }


    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_UPLOAD_RFID_DST_SRC_ID)
    {
        if (data_len == 12 + 1)
        {
            ROS_INFO("get CAN_SOURCE_ID_UPLOAD_RFID_DST_SRC_ID");
            CAN_ID_UNION id;
            uint8_t serial_num = msg->Data[msg->DataLen - 1];
            id.CanID_Struct.ACK = 1;
            id.CanID_Struct.SourceID = CAN_SOURCE_ID_UPLOAD_RFID_DST_SRC_ID;
            this->ack_mcu_upload(dev_id, id, serial_num);
//            for(uint8_t i = 0; i < msg->DataLen; i++)
//            {
//                ROS_INFO("data[%d]: 0x%x", i, msg->Data[i]);
//            }
            uint32_t info = msg->Data[3] |  (msg->Data[2] << 8) | (msg->Data[1] << 16) | (msg->Data[0] << 24);
            uint32_t dst_info = msg->Data[7] |  (msg->Data[6] << 8) | (msg->Data[5] << 16) | (msg->Data[4] << 24);
            uint32_t src_info = msg->Data[11] |  (msg->Data[10] << 8) | (msg->Data[9] << 16) | (msg->Data[8] << 24);
            ROS_INFO("get rfid info %d", info);
            ROS_INFO("get dst id    0x%x", dst_info);
            ROS_INFO("get src id    0x%x", src_info);
            //this->pub_rfid_info(dev_id, 1, 0);
            report_rfid_info(rfid_type, index, 1, info, dst_info, src_info);
        }
    }


}

