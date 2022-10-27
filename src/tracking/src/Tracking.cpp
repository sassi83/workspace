/******************************************************************************
 *  @file       Tracking.cpp
 *
 *  <!-- Start of section for manual description -->
 *  @brief      Node for get the position of courier
 *  input serial stream from tracker
 *  publish afius_msgs::msg::Track
 *  <!-- End of section for manual description -->
 *
 *  @author     Stefan Sass <stefan.sass@ovgu.de>
 *  @author

 ******************************************************************************/
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "rtls_driver/rtls_driver.hpp"
#include "serial_communication/serial.hpp"

// Custom Message
#include "afius_msgs/msg/track.hpp"

#include "tracking.h"


using std::placeholders::_1;


class Tracker : public rclcpp::Node
{
public:
    Tracker()
    : Node("tracking")
    {
        publisher_position_ = this->create_publisher<afius_msgs::msg::Track>("tracking", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Tracker::process_tracking, this));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    virtual ~Tracker()
    {}

    virtual void onInit()
    {

        getParameters();
        setSerial();
        setupDevice();
    }

    void closeTrackerStream()
    {
        terabee::RtlsDevice rtls_device(serial_port);
        rtls_device.disableTrackerStream();
        rtls_device.stopReadingStream();
        RCLCPP_INFO(this->get_logger(),"[Tracking]: Disable tracker stream ");
    }

private:

    // Publisher
    rclcpp::Publisher<afius_msgs::msg::Track>::SharedPtr publisher_position_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // serial input and msgs
    std::shared_ptr<terabee::serial_communication::ISerial> serial_port;
    std::string tracker_line;

    // parameters and variables
    bool publish_tf;
    std::string frame;
    std::string ref_frame;
    std::string portname;
    int serial_timeout;

    // ros2 + tf2 msgs
    afius_msgs::msg::Track track_msg;
    geometry_msgs::msg::TransformStamped transform_st;

    // load parameter from config file or default values
    void getParameters()
    {
        RCLCPP_INFO(this->get_logger(), "getParameters...");
        this->get_parameter_or("publish_tf", publish_tf, Tracking::PUBLISH_TF);
        this->get_parameter_or("ref_frame", ref_frame, Tracking::REF_FRAME);
        this->get_parameter_or("frame", frame, Tracking::FRAME);
        this->get_parameter_or("portname", portname, Tracking::PORTNAME);
        this->get_parameter_or("serial_timeout", serial_timeout, Tracking::SERIAL_TIMEOUT);
        RCLCPP_INFO_STREAM(this->get_logger(), "[PredictionCV]: Params loaded: publish_tf = "<< publish_tf
         << " ref_frame = "<< ref_frame << " frame = "<< frame<< " portname = "<< portname<< " serial_timeout = "<< serial_timeout);
    }

    // config serial port
    void setSerial()
    {
        RCLCPP_INFO(this->get_logger(), "setSerial...");
        serial_port = std::make_shared<terabee::serial_communication::Serial>(portname);
        serial_port->setBaudrate(115200);
        serial_port->setTimeout(std::chrono::milliseconds(serial_timeout));
        serial_port->open();
        if (!serial_port->isOpen())
        {
            RCLCPP_ERROR(this->get_logger(),"[Tracking]: Failed to open serial port!");
        }
    }

    // config tracker device
    void setupDevice()
    {
        RCLCPP_INFO(this->get_logger(), "setDevice...");
        terabee::RtlsDevice rtls_device(serial_port);
        rtls_device.disableTrackerStream();
        serial_port->flushInput();
        rtls_device.setDevice(terabee::RtlsDevice::device_type::tracker, 1);
        rtls_device.setLabel(0x04);
        rtls_device.setUpdateTime(1);
        rtls_device.setNetworkId(0x01);
        rtls_device.setTrackerMessageShort();
        rtls_device.enableLED();
        //rtls_device.requestConfig();
        terabee::RtlsDevice::config_t device_configuration = rtls_device.getConfig();
        RCLCPP_INFO_STREAM(this->get_logger(),"[Tracking]: Tracker config " << rtls_device.serializeConfig());
        rtls_device.enableTrackerStream();
        RCLCPP_INFO(this->get_logger(),"[Tracking]: Enable tracker stream ");
    }

    // methode for process and publish track msgs and tf2 msgs
    void process_tracking()
    {
        track_msg.is_valid_position = true;
        // read serial
        tracker_line = serial_port->readline();
        tracker_line.erase(std::find(tracker_line.begin(), tracker_line.end(), '\0'),
                           tracker_line.end());
        std::vector<std::string> fields = Tracking::splitString(tracker_line, ',');

        // fill header
        track_msg.header.stamp = this->get_clock()->now();
        track_msg.header.frame_id = frame;

        // set valid position
        track_msg.is_valid_position = false;

        // check for valid postition and fill msgs
        if (fields.size() == Tracking::TRACKER_STREAM_SHORT_LEN &&
                fields[0] != "N/A" &&
                fields[1] != "N/A" &&
                fields[2] != "N/A" &&
                fields[3] != "N/A")
        {

            track_msg.is_valid_position = true;
            //convert to ros msgs

            track_msg.x = Tracking::MM_TO_M_FACTOR*std::stod(fields[1]);
            track_msg.y = Tracking::MM_TO_M_FACTOR*std::stod(fields[2]);
            track_msg.z = Tracking::MM_TO_M_FACTOR*std::stod(fields[3]);

            RCLCPP_INFO_STREAM(this->get_logger(), "[Tracking]: track_msg.x: "<< track_msg.x
            << " track_msg.y: " << track_msg.y <<" track_msg.z: "<< track_msg.z);


            // publish ros msgs
            publisher_position_->publish(track_msg);

            // publish tf2
            if (publish_tf)
            {
                transform_st.header.stamp = this->get_clock()->now();
                transform_st.header.frame_id = ref_frame;
                transform_st.child_frame_id = frame;
                transform_st.transform.translation.x = track_msg.x;
                transform_st.transform.translation.y = track_msg.y;
                transform_st.transform.translation.z = track_msg.z;
                transform_st.transform.rotation.x = 0.0;
                transform_st.transform.rotation.y = 0.0;
                transform_st.transform.rotation.z = 0.0;
                transform_st.transform.rotation.w = 1.0;

                // publish tf2
                tf_broadcaster_->sendTransform(transform_st);
            }
        }
    }
};







int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tracker>();
    node->onInit();
    rclcpp::spin(node);
    node->closeTrackerStream();
    rclcpp::shutdown();
    return 0;
}