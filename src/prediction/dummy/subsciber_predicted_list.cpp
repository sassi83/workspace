#include <memory>

#include "rclcpp/rclcpp.hpp"

// Custom Message
#include "afius_msgs/msg/predicted_object_list.hpp"


using std::placeholders::_1;

class PredictedListSubscriber : public rclcpp::Node
{
public:
    PredictedListSubscriber()
            : Node("subscriber_predicted_list"){
        // QoS
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        // subscriber for predicted list
        subscription_ = this->create_subscription<afius_msgs::msg::PredictedObjectList>(
                "prediction_list", default_qos, std::bind(&PredictedListSubscriber::process_predicted_list, this, _1));
    }

private:
    // Subsciber shared pointer
    rclcpp::Subscription<afius_msgs::msg::PredictedObjectList>::SharedPtr subscription_;

    void process_predicted_list(const afius_msgs::msg::PredictedObjectList & msg) const
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received new predicted list of "<<msg.predicted_object_list.size() << " objects");

        if(msg.predicted_object_list.size() > 0)
        {
            // plot start end
            RCLCPP_INFO_STREAM(this->get_logger(), "x1 "
                    << msg.predicted_object_list[0].object_trajectory_list[0].state_list[0].x <<
                    " x30 " << msg.predicted_object_list[0].object_trajectory_list[0].state_list[29].x <<
                    " x60 " << msg.predicted_object_list[0].object_trajectory_list[0].state_list[59].x);

            RCLCPP_INFO_STREAM(this->get_logger(), "y1 "
                    << msg.predicted_object_list[0].object_trajectory_list[0].state_list[0].y <<
                    " y30 " << msg.predicted_object_list[0].object_trajectory_list[0].state_list[29].y <<
                    " y60 " << msg.predicted_object_list[0].object_trajectory_list[0].state_list[59].y);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PredictedListSubscriber>());
    rclcpp::shutdown();
    return 0;
}