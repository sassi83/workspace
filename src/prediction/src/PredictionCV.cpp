/******************************************************************************
 *  @file       PredictionCV.cpp
 *
 *  <!-- Start of section for manual description -->
 *  @brief      Node for constant velocity trajectory prediction,
 *  subscribe afius_msgs::msg::ObjectList,
 *  publish afius_msgs::msg::PredictedObjectList
 *  <!-- End of section for manual description -->
 *
 *  @author     Stefan Sass <stefan.sass@ovgu.de>
 *  @author

 ******************************************************************************/

#include "prediction.hpp"

using std::placeholders::_1;


/// class for prediction cv node
class PredCV : public rclcpp::Node {
public:
    // constructor
    PredCV()
    : Node("prediction_cv")
    {
        // QoS
        auto defaultQoS = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // subscriber for object list
        subscription_ = this->create_subscription<afius_msgs::msg::ObjectList>("object_list", defaultQoS,
                std::bind(&PredCV::processObjectList, this, _1));

        // publisher for predicted object list
        publisher_ = this->create_publisher<afius_msgs::msg::PredictedObjectList>("prediction_list", 10);

        // get parameter
        getParameters();

        //set parameters
        predCV.setNumTimesteps(numTimesteps);
        predCV.setTimestep(timestep);
    }

private:
    // Publisher shared pointer
    rclcpp::Publisher<afius_msgs::msg::PredictedObjectList>::SharedPtr publisher_;

    // Subsciber shared pointer
    rclcpp::Subscription<afius_msgs::msg::ObjectList>::SharedPtr subscription_;

    //create prediction_cv object
    Prediction::CV predCV;

    // predicted object list
    afius_msgs::msg::PredictedObjectList predictionObjectList;

    // parameter
    size_t numTimesteps;
    double timestep;

    // load parameter from config file or default values
    void getParameters()
    {
        RCLCPP_INFO(this->get_logger(), "GetParameters...");
        this->get_parameter_or<size_t>("num_timesteps", numTimesteps, 60);
        this->get_parameter_or("timestep", timestep, 0.05);
        RCLCPP_INFO_STREAM(this->get_logger(), "Params loaded: numTimesteps = "
        << numTimesteps << " timestep = "<< timestep);
    }

    // process incoming object list
    void processObjectList(const afius_msgs::msg::ObjectList::SharedPtr objectList) {

        RCLCPP_INFO_STREAM(this->get_logger(), "Received new object list of "<<objectList->dynamic_object_list.size() << " objects");

        // prediction_cv
        predCV.predict(objectList, predictionObjectList);

        // publish m_prediction_object_list
        publisher_->publish(predictionObjectList);
    }
};




int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PredCV>());
    rclcpp::shutdown();
    return 0;
}
