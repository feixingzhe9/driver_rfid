
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <boost/thread/mutex.hpp>
#include "std_msgs/UInt16MultiArray.h"

#define RFID_MAX_NUM                    4

#define RFID_CAN_MAC_SRC_ID_BASE                0x80


#define CAN_SOURCE_ID_READ_VERSION              0x01
#define CAN_SOURCE_ID_RFID_INFO                 0x83


#define OLD_CAN_SOURCE_ID_RFID_VERSION          0x01
#define OLD_CAN_SOURCE_ID_RFID_UID              0x81
#define OLD_CAN_SOURCE_ID_RFID_TYPE             0x82
#define OLD_CAN_SOURCE_ID_RFID_DATA             0x83

#define CAN_SOURCE_ID_UPLOAD_RFID_LEAVE         0xb2

#define PROTOCOL_DATA_LEN_MAX   32

typedef struct
{
    uint8_t len_min;
    uint8_t len_max;
}expect_len_t;

typedef struct
{
    expect_len_t len;
    uint8_t data[PROTOCOL_DATA_LEN_MAX];
}expect_data_t;

typedef struct
{
    uint8_t serial_num;
    expect_len_t expect_len;
    mrobot_msgs::vci_can can_msg;
    uint8_t retry_cnt;
}protocol_t;

typedef struct
{
    uint8_t serial_num;
    mrobot_msgs::vci_can can_msg;
}protocol_ack_t;

class DriverRFID
{
    public:
        DriverRFID(bool log_on = false)
        {
            sub_from_can = n.subscribe("can_to_driver_rfid", 2, &DriverRFID::rcv_from_can_node_callback, this);
            old_sub_from_can = n.subscribe("rx_rfid_node", 2, &DriverRFID::old_rcv_from_can_node_callback, this);
            pub_to_can = n.advertise<mrobot_msgs::vci_can>("driver_rfid_to_can", 10);
            old_pub_to_can = n.advertise<mrobot_msgs::vci_can>("tx_rfid_node", 10);
            rfid_pub = n.advertise<std_msgs::UInt16MultiArray>("/driver_rfid/pub_info", 2);

            old_rfid_pub = n.advertise<std_msgs::String>("rfid_pub", 2);

            protocol_vector.clear();
            protocol_ack_vector.clear();
            protocol_serial_num = 0;
old_rfid_uid.clear();
        }

        can_long_frame  long_frame;
        boost::mutex mtx;


        vector<protocol_t> protocol_vector;
        vector<protocol_ack_t> protocol_ack_vector;


        void pub_msg_to_can(mrobot_msgs::vci_can can_msg);

        int get_version(uint8_t id, uint8_t type);

    private:
        ros::NodeHandle n;
        ros::Publisher rfid_pub;
        ros::Publisher old_rfid_pub;
        ros::Subscriber sub_from_can;
        ros::Subscriber old_sub_from_can;
        ros::Publisher pub_to_can;
        ros::Publisher old_pub_to_can;

        uint8_t protocol_serial_num;
        std::string hex_2_str(uint8_t hex);
        int ack_mcu_upload(uint8_t dev_id, CAN_ID_UNION id, uint8_t serial_num);
        uint8_t get_dev_id_by_src_id(uint8_t src_id);
        void rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg);
        void old_rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg);
        void pub_rfid_info(uint8_t dev_id, uint8_t type, uint16_t data);
        void old_pub_rfid_info(std::string uid, std::string type, std::string data);

        std::string old_rfid_uid;
        std::string old_rfid_type;
        std::string old_rfid_data;

        std::string hw_version[RFID_MAX_NUM];
        std::string sw_version[RFID_MAX_NUM];
        std::string protocol_version[RFID_MAX_NUM];

        std::string rfid_MCU_version_param[RFID_MAX_NUM] = {"mcu_rfid_0_version","mcu_rfid_1_version","mcu_rfid_2_version","mcu_rfid_3_version"
                                                           //, "mcu_rfid_4_version","mcu_rfid_5_version","mcu_rfid_6_version","mcu_rfid_7_version"
                                                            };



};
