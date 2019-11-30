
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <boost/thread/mutex.hpp>
#include "std_msgs/UInt16MultiArray.h"

#define RFID_MAX_NUM                    0x0f

#define RFID_CAN_MAC_SRC_ID_BASE                0x80


#define CAN_SOURCE_ID_READ_VERSION              0x01
#define CAN_SOURCE_ID_RFID_INFO                 0x83


#define OLD_CAN_SOURCE_ID_RFID_VERSION          0x01
#define OLD_CAN_SOURCE_ID_RFID_UID              0x81
#define OLD_CAN_SOURCE_ID_RFID_TYPE             0x82
#define OLD_CAN_SOURCE_ID_RFID_DATA             0x83

#define CAN_SOURCE_ID_UPLOAD_RFID_DEPART        0xb2
#define CAN_SOURCE_ID_UPLOAD_RFID_DST_SRC_ID    0xc0

#define PROTOCOL_DATA_LEN_MAX   32



#define CABINET_DETECT_RFID_NUM         (4)
#define DST_SRC_INFO_RFID_NUM           (1)
#define AUTH_RFID_NUM                   (1)

#if CABINET_DETECT_RFID_NUM > 9
    #ERROR  "ERROR: cabinet detection rfid num Could Not > 9 !  Check rfid num !"
#endif

#if DST_SRC_INFO_RFID_NUM > 4
    #ERROR  "ERROR: dst src info rfid num Could Not > 4 !  Check rfid num !"
#endif

#if AUTH_RFID_NUM > 3
    #ERROR  "ERROR: auth rfid num Could Not > 3 !  Check rfid num !"
#endif

#if (CABINET_DETECT_RFID_NUM + DST_SRC_INFO_RFID_NUM + AUTH_RFID_NUM) > 16
    #ERROR  "ERROR: rfid total num Could Not > 16 !  Check rfid num !"
#endif

//define rfid start and end of dip_swich_value
#define CABINET_DETECT_RFID_ID_START    1
#define CABINET_DETEDT_RFID_ID_END      CABINET_DETECT_RFID_NUM

#define DST_SRC_INFO_RFID_ID_START      10
#define DST_SRC_INFO_RFID_ID_END        (DST_SRC_INFO_RFID_ID_START + DST_SRC_INFO_RFID_NUM - 1)

#define AUTH_RFID_ID_START              14

#if AUTH_RFID_NUM < 3
    #define AUTH_RFID_ID_END                (AUTH_RFID_ID_START + AUTH_RFID_NUM - 1)
#else
    #define AUTH_RFID_ID_END                0
#endif

enum
{
    RFID_TYPE_CABINET_DETECT = 1,
    RFID_TYPE_DST_SRC_INFO,
    RFID_TYPE_AUTH,
    RFID_TYPE_NONE,
};

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
            report_rfid_info_pub = n.advertise<std_msgs::String>("/driver_rfid/report_info", 2);

            old_rfid_pub = n.advertise<std_msgs::String>("rfid_pub", 2);

            protocol_vector.clear();
            protocol_ack_vector.clear();
            protocol_serial_num = 0;
            old_rfid_uid.clear();
            //rfid_type = 0;
        }

        can_long_frame  long_frame;
        boost::mutex mtx;


        vector<protocol_t> protocol_vector;
        vector<protocol_ack_t> protocol_ack_vector;
        //uint8_t rfid_type;

        void pub_msg_to_can(mrobot_msgs::vci_can can_msg);

        void get_auth_rfid_version(uint8_t type);
        void get_dst_src_info_rfid_version(uint8_t type);
        void get_cabinet_detection_rfid_version(uint8_t type);

    private:
        ros::NodeHandle n;
        ros::Publisher rfid_pub;
        ros::Publisher old_rfid_pub;
        ros::Subscriber sub_from_can;
        ros::Subscriber old_sub_from_can;
        ros::Publisher pub_to_can;
        ros::Publisher old_pub_to_can;
        ros::Publisher report_rfid_info_pub;

        uint8_t protocol_serial_num;

        int get_version(uint8_t id, uint8_t type);

        std::string hex_2_str(uint8_t hex);
        int ack_mcu_upload(uint8_t dev_id, CAN_ID_UNION id, uint8_t serial_num);
        uint8_t get_dev_id_by_src_id(uint8_t src_id);
        void rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg);
        void old_rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg);
        void pub_rfid_info(uint8_t dev_id, uint8_t type, uint16_t data);
        void old_pub_rfid_info(std::string uid, std::string type, std::string data);

        uint8_t get_rfid_type_by_dev_id(uint8_t dev_id);
        uint8_t get_rfid_type_index(uint8_t dev_id);
        int report_rfid_info(uint8_t rfid_type, uint8_t index, uint8_t act, uint32_t rfid_info, uint32_t dst_info, uint32_t src_info);

        std::string get_version_param(uint8_t rfid_type, uint8_t index, uint8_t version_type);

        std::string old_rfid_uid;
        std::string old_rfid_type;
        std::string old_rfid_data;

        std::string hw_version[RFID_MAX_NUM];
        std::string sw_version[RFID_MAX_NUM];
        std::string protocol_version[RFID_MAX_NUM];

        std::string rfid_MCU_version_param[RFID_MAX_NUM] = {"mcu_rfid_0_version","mcu_rfid_1_version","mcu_rfid_2_version","mcu_rfid_3_version"
                                                           //, "mcu_rfid_4_version","mcu_rfid_5_version","mcu_rfid_6_version","mcu_rfid_7_version"
                                                            };
        std::string auth_rfid_mcu_version_param[3] = {"mcu_auth_rfid_0_version", "mcu_auth_rfid_1_version", "mcu_auth_rfid_2_version"};

        std::string cabinet_rfid_mcu_version_param[9] = {"mcu_cabinet_rfid_0_version", "mcu_cabinet_rfid_1_version", \
                                                         "mcu_cabinet_rfid_2_version", "mcu_cabinet_rfid_3_version", \
                                                         "mcu_cabinet_rfid_4_version", "mcu_cabinet_rfid_5_version", \
                                                         "mcu_cabinet_rfid_6_version", "mcu_cabinet_rfid_7_version", \
                                                         "mcu_cabinet_rfid_8_version"};

        std::string dst_src_rfid_mcu_version_param[4] = {"mcu_src_dst_rfid_0_version", "mcu_src_dst_rfid_1_version"\
                                                      "mcu_src_dst_rfid_2_version", "mcu_src_dst_rfid_3_version"};

};
