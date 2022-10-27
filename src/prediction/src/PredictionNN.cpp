/******************************************************************************
 *  @file       PredictionNN.cpp
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
#include <time.h>

#include "prediction.hpp"

using std::placeholders::_1;


/// class for prediction cv node
class PredNN : public rclcpp::Node {
public:
    // constructor
    PredNN()
    : Node("prediction_cv")
    {
        // QoS
        auto defaultQoS = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // subscriber for object list
        subscription_objectlist_ = this->create_subscription<afius_msgs::msg::ObjectList>("object_list", defaultQoS,
                std::bind(&PredNN::processObjectList, this, _1));

        // subscriber for object list
        subscription_egostate_ = this->create_subscription<afius_msgs::msg::EgoState>("ego_state", defaultQoS,
                                                                                          std::bind(&PredNN::processEgoState, this, _1));

        // subscriber for polygon list
        subscription_polygon_ = this->create_subscription<afius_msgs::msg::PolygonList>("poly_list", defaultQoS,
                                                                                      std::bind(&PredNN::processPolyList, this, _1));

        // publisher for predicted object list
        publisher_ = this->create_publisher<afius_msgs::msg::PredictedObjectList>("prediction_list", 10);

        // get parameter
        getParameters();

        // Init prediction Model without gpu settings
        // predNN.onInit((modelPath).c_str());

        // Init prediction Model with gpu settings
        predNN.onInit((modelPath).c_str(), gpuID, gpuMemFrac);
    }

private:
    // Publisher shared pointer
    rclcpp::Publisher<afius_msgs::msg::PredictedObjectList>::SharedPtr publisher_;

    // Subsciber shared pointer
    rclcpp::Subscription<afius_msgs::msg::ObjectList>::SharedPtr subscription_objectlist_;
    rclcpp::Subscription<afius_msgs::msg::EgoState>::SharedPtr subscription_egostate_;
    rclcpp::Subscription<afius_msgs::msg::PolygonList>::SharedPtr subscription_polygon_;

    // parameter
    std::string modelPath;
    int gpuID;
    double gpuMemFrac;

    // ros messages
    afius_msgs::msg::EgoState egoState;
    afius_msgs::msg::PolygonList polyList;
    afius_msgs::msg::PredictedObjectList predictionObjectList;

    // Model for prediction
    Prediction::NN predNN;

    // load parameter from config file or default values
    void getParameters()
    {
        RCLCPP_INFO(this->get_logger(), "GetParameters...");
        this->get_parameter_or<std::string>("model_path", modelPath,
                                            "/home/afius/workspace/src/prediction/model/aura_model_gaus_non_para.pb");
        this->get_parameter_or<int>("gpu_id", gpuID, 0);
        this->get_parameter_or<double>("gpu_memory_fraction", gpuMemFrac, 0.9);

        RCLCPP_INFO_STREAM(this->get_logger(), "Params loaded: modelPath = "<< modelPath
        <<" gpu_id = " << gpuID <<" gpu memory fraction = " << gpuMemFrac);
    }


    // process incoming object list
    void processObjectList(const afius_msgs::msg::ObjectList::SharedPtr objectList) {

        RCLCPP_INFO_STREAM(this->get_logger(), "Received new object list of "
        << objectList->dynamic_object_list.size() << " objects");

        // time measurement
        double t = 0.0, tstart;
        tstart = clock();

        // process msgs and prediction
        predNN.processData(* objectList, egoState, polyList, predictionObjectList);

        // time measurement
        t += clock() - tstart;
        t = t/CLOCKS_PER_SEC;  // rescale to seconds
        RCLCPP_INFO_STREAM(this->get_logger(), "time = " << t << " sec.");

        // publish m_prediction_object_list
        publisher_->publish(predictionObjectList);
    }

    // process incoming ego state
    void processEgoState(const afius_msgs::msg::EgoState::SharedPtr egoState_) {

        RCLCPP_INFO_STREAM(this->get_logger(), "Received new ego state");
        egoState = *egoState_;
    }

    // process incoming poly list
    void processPolyList(const afius_msgs::msg::PolygonList::SharedPtr polyList_) {

        RCLCPP_INFO_STREAM(this->get_logger(), "Received new ego state");
        polyList = *polyList_;
    }

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PredNN>());
    rclcpp::shutdown();
    return 0;
}
