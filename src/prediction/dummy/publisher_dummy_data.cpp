#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

// Custom Message
#include "afius_msgs/msg/object_list.hpp"
#include "afius_msgs/msg/ego_state.hpp"
#include "afius_msgs/msg/polygon_list.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class DummyDataPublisher : public rclcpp::Node
{
public:
    DummyDataPublisher()
  : Node("publisher_dummy_data"), count(0)
  {
    publisher_object_list = this->create_publisher<afius_msgs::msg::ObjectList>("object_list", 10);
    publisher_ego_state = this->create_publisher<afius_msgs::msg::EgoState>("ego_state", 10);
    publisher_poly_list = this->create_publisher<afius_msgs::msg::PolygonList>("poly_list", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&DummyDataPublisher::process_object_list, this));

    // create path parameter for loading dummy_trajectory.txt
    this->declare_parameter<std::string>("dummy_trajectory_path", "/home/afius/workspace/src/prediction/dummy/dummy_trajectory.txt");

    // load dummy data
    float x;
    int element = 0;
    double frame_nr = 0;

    std::vector<std::vector<float>> frame;
    std::vector<float> line;
    std::ifstream inFile;

    // load parameter for loading dummy_trajectory.txt
    this->get_parameter("dummy_trajectory_path", dummy_trajectory_path_);
    inFile.open(dummy_trajectory_path_.c_str());

    if (!inFile) {
        RCLCPP_INFO_STREAM(this->get_logger(), "path = "<< dummy_trajectory_path_);
        RCLCPP_ERROR(this->get_logger(), "Unable to open file");
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "File loaded from path = "<< dummy_trajectory_path_);
    }

    while (inFile >> x)
    {
      if(element == 0 and frame_nr != x) {
          frame_nr = x;
          trajectory.push_back(frame);
          frame.clear();
      }
      line.push_back(x);
      element++;
      if(element == 10)
      {
          element=0;
          frame.push_back(line);
          line.clear();
      }
    }
    inFile.close();
  }

private:
    // path to dummy_trajectory.txt
    std::string dummy_trajectory_path_;

    // Publisher shared pointer
    rclcpp::Publisher<afius_msgs::msg::ObjectList>::SharedPtr publisher_object_list;
    rclcpp::Publisher<afius_msgs::msg::EgoState>::SharedPtr publisher_ego_state;
    rclcpp::Publisher<afius_msgs::msg::PolygonList>::SharedPtr publisher_poly_list;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // variables
    std::vector<std::vector<std::vector<float>>> trajectory;

    // dummy data EgoState
    afius_msgs::msg::EgoState m_ego_msg;

    // dummy data
    afius_msgs::msg::ObjectList m_object_list_msg;

    int count;
    int number_static_objects;

    // methode for creating dummy data -> object list and ego state
    void process_object_list()
    {
        //#################################################### objects ###############################################
        // random number of static obstacles
        number_static_objects = rand() % 128;

        RCLCPP_INFO_STREAM(this->get_logger(), "dummy id/size = "<< count <<"/" <<trajectory.size()<<" Number of dynamic objects "<<trajectory[count].size());

        // fill header of object list
        m_object_list_msg.header.stamp = this->get_clock()->now();
        m_object_list_msg.header.frame_id = count;

        // resize dynamic/static object list
        m_object_list_msg.dynamic_object_list.resize(trajectory[count].size());
        m_object_list_msg.static_object_list.resize(number_static_objects);

        int mean_x = 0;
        int mean_y = 0;

        // dynamic_obstacles
        for(size_t i = 0; i < trajectory[count].size(); i++) {
            mean_x += trajectory[count][0][3];
            mean_y += trajectory[count][0][4];

            m_object_list_msg.dynamic_object_list[i].stamp = this->get_clock()->now();
            m_object_list_msg.dynamic_object_list[i].id = trajectory[count][0][1];
            m_object_list_msg.dynamic_object_list[i].x_centroid = trajectory[count][0][3];
            m_object_list_msg.dynamic_object_list[i].y_centroid = trajectory[count][0][4];
            m_object_list_msg.dynamic_object_list[i].z_centroid = rand() % 2;
            m_object_list_msg.dynamic_object_list[i].width = trajectory[count][0][7];
            m_object_list_msg.dynamic_object_list[i].length = trajectory[count][0][6];
            m_object_list_msg.dynamic_object_list[i].height = rand() % 2;
            m_object_list_msg.dynamic_object_list[i].category_list.resize(4);
            m_object_list_msg.dynamic_object_list[i].category_list[0].category = trajectory[count][0][2];
            m_object_list_msg.dynamic_object_list[i].category_list[0].category_probability = rand() % 100;
            for(int j = 0; j < 6; j++) m_object_list_msg.dynamic_object_list[i].covariance[j] = rand() % 1;
            m_object_list_msg.dynamic_object_list[i].state.x = 100  + count + rand() % 100;
            m_object_list_msg.dynamic_object_list[i].state.y = 60  + count + rand() % 100;
            m_object_list_msg.dynamic_object_list[i].state.z = rand() % 120;
            m_object_list_msg.dynamic_object_list[i].state.yaw = rand() / (RAND_MAX + 1.);
            m_object_list_msg.dynamic_object_list[i].state.course_rate = rand() / (RAND_MAX + 1.);
            m_object_list_msg.dynamic_object_list[i].state.course_angle = rand() / (RAND_MAX + 1.);
            m_object_list_msg.dynamic_object_list[i].state.velocity = rand() % 5;
            for(int j = 0; j < 15; j++) m_object_list_msg.dynamic_object_list[i].state.covariance[j] = rand() % 1;
        }

        mean_x /= trajectory[count].size();
        mean_y /= trajectory[count].size();

        // static_obstacles
        for(int i = 0; i < number_static_objects; i++){
            m_object_list_msg.static_object_list[i].stamp = this->get_clock()->now();
            m_object_list_msg.static_object_list[i].id = i + 1 + count;
            m_object_list_msg.static_object_list[i].x_centroid = trajectory[count][0][3] + rand() % 20;
            m_object_list_msg.static_object_list[i].y_centroid = trajectory[count][0][4] + rand() % 20;
            m_object_list_msg.static_object_list[i].z_centroid = rand() % 2;
            m_object_list_msg.static_object_list[i].width=rand() % 10;
            m_object_list_msg.static_object_list[i].length=rand() % 10;
            m_object_list_msg.static_object_list[i].height=rand() % 10;
            m_object_list_msg.static_object_list[i].category_list.resize(4);
            m_object_list_msg.static_object_list[i].category_list[0].category = rand() % 255;
            m_object_list_msg.static_object_list[i].category_list[0].category_probability = rand() % 100;
            for(int j = 0; j < 6; j++) m_object_list_msg.static_object_list[i].covariance[0] = rand() % 1;
            m_object_list_msg.static_object_list[i].state.x= 100  + count + rand() % 100;
            m_object_list_msg.static_object_list[i].state.y= 60  + count + rand() % 100;
            m_object_list_msg.static_object_list[i].state.z=rand() % 120;
            m_object_list_msg.static_object_list[i].state.yaw=rand() / (RAND_MAX + 1.);
            m_object_list_msg.static_object_list[i].state.course_rate=rand() / (RAND_MAX + 1.);
            m_object_list_msg.static_object_list[i].state.course_angle = rand() / (RAND_MAX + 1.);
            m_object_list_msg.static_object_list[i].state.velocity=0;
            for(int j = 0; j < 15; j++) m_object_list_msg.static_object_list[i].state.covariance[j] = rand() % 1;
        }
        // publish object list
        publisher_object_list->publish(m_object_list_msg);

        //#################################################### ego state ###############################################
        m_ego_msg.x = mean_x;
        m_ego_msg.y = mean_y;
        m_ego_msg.ax = rand() % 1;
        m_ego_msg.ay = rand() % 1;
        m_ego_msg.yaw = rand() % 3;
        m_ego_msg.yawrate = rand() % 2;
        m_ego_msg.velocity = rand() % 10;
        m_ego_msg.sideslip = rand() % 1;
        m_ego_msg.steer_angle = rand() % 1;

        // publish dummy data
        publisher_ego_state->publish(m_ego_msg);


        //#################################################### polygons ###############################################

        /* class id
         * 84 -> Fußgängerüberweg
         * 85 -> Radwegmarkierung
         * 86 -> Gehwegmarkierung
         * 87 -> Gemeinsamer Geh- und Radweg Markierung
         * 88 -> Getrennter Rad- und Gehweg, Radweg links Markierung
         * 89 -> Getrennter Rad- und Gehweg, Radweg rechts Markierung
         * 90 -> Haltelinie
         * ** -> Fahrstreifen- und Fahrbahnbegrenzung (295) als Einzelobjekt gelabelt
         * */

        int poly_size = 4;
        afius_msgs::msg::PolygonList polygon_list;
        polygon_list.polygon_list.resize(poly_size);

        polygon_list.polygon_list[0].category.category = 86;
        polygon_list.polygon_list[1].category.category = 86;
        polygon_list.polygon_list[2].category.category = 85;
        polygon_list.polygon_list[3].category.category = 85;

        for(int id = 0; id < poly_size; id++)
        {
            polygon_list.polygon_list[id].id = count + id;
            polygon_list.polygon_list[id].point_list.resize(50);
        }

        for (int pixel = 0; pixel < 50; pixel++)
        {
            polygon_list.polygon_list[0].point_list[pixel].x = m_ego_msg.x - 100;
            polygon_list.polygon_list[0].point_list[pixel].y = pixel;
            polygon_list.polygon_list[1].point_list[pixel].x = m_ego_msg.x + 100;
            polygon_list.polygon_list[1].point_list[pixel].y = pixel;

            polygon_list.polygon_list[2].point_list[pixel].x = m_ego_msg.x - 50;
            polygon_list.polygon_list[2].point_list[pixel].y = pixel;
            polygon_list.polygon_list[3].point_list[pixel].x = m_ego_msg.x + 50;
            polygon_list.polygon_list[3].point_list[pixel].y = pixel;
        }

        // publish polygon list
        publisher_poly_list->publish(polygon_list);


        count++;
        if(count >= (int)trajectory.size())
        {
            count = 0;
        }
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyDataPublisher>());
    rclcpp::shutdown();
    return 0;
}
