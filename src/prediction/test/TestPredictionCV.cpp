/******************************************************************************
 *  @file       TestPredictionCV.cpp
 *
 *  <!-- Start of section for manual description -->
 *  @brief      UnitTest for PredictionCV
 *  <!-- End of section for manual description -->
 *
 *  @author      Stefan Sass <stefan.sass@ovgu.de>
 *
 ******************************************************************************/
#include "gtest/gtest.h"

#include "prediction.hpp"


/// class for mocking ObjectList
class MockObjectList
{
public:
    MockObjectList()
    {
        fillObjectList();
    }

    // get dummy object list data
    afius_msgs::msg::ObjectList getObjectList(){return objectList;}

    // manipulate x,y,velocity and yaw for test prediction_cv
    void setPara(double state_x, double state_y, double velocity, double yaw)
    {
        objectList.dynamic_object_list[0].state.x = state_x;
        objectList.dynamic_object_list[0].state.y = state_y;
        objectList.dynamic_object_list[0].state.yaw = yaw;
        objectList.dynamic_object_list[0].state.velocity = velocity;
    }

private:
    // dummy data
    afius_msgs::msg::ObjectList objectList;
    int numberDynamicObjects = 1;
    int numberStaticObjects = 2;


    // methode for creating dummy data -> object list and ego state
    void fillObjectList()
    {
        //#################################################### objects ###############################################

        // fill header of object list
        objectList.header.stamp.sec = 1662975851;
        objectList.header.stamp.nanosec = 321584163;
        objectList.header.frame_id = "164";

        // resize dynamic/static object list
        objectList.dynamic_object_list.resize(numberDynamicObjects);
        objectList.static_object_list.resize(numberStaticObjects);

        // dynamic_obstacles
        for(int i = 0; i < numberDynamicObjects; i++) {

            objectList.dynamic_object_list[i].stamp.sec = 1662975850;
            objectList.dynamic_object_list[i].stamp.nanosec = 321584161;
            objectList.dynamic_object_list[i].id = 1;
            objectList.dynamic_object_list[i].x_centroid = 378.3559875488281;
            objectList.dynamic_object_list[i].y_centroid = 109.8290023803711;
            objectList.dynamic_object_list[i].z_centroid = 0.75;
            objectList.dynamic_object_list[i].width = 0.8939999938011169;
            objectList.dynamic_object_list[i].length = 1.090999960899353;
            objectList.dynamic_object_list[i].height = 1.8;
            objectList.dynamic_object_list[i].category_list.resize(1);
            objectList.dynamic_object_list[i].category_list[0].category = 4;
            objectList.dynamic_object_list[i].category_list[0].category_probability = rand() % 100;
            for(int j = 0; j < 6; j++) objectList.dynamic_object_list[i].covariance[j] = rand() % 1;
            objectList.dynamic_object_list[i].state.x = 253.0;
            objectList.dynamic_object_list[i].state.y = 183.0;
            objectList.dynamic_object_list[i].state.z = 21.5;
            objectList.dynamic_object_list[i].state.yaw = 0.8328700661659241;
            objectList.dynamic_object_list[i].state.course_rate = 0.20446757972240448;
            objectList.dynamic_object_list[i].state.course_angle = 0.34622249007225037;
            objectList.dynamic_object_list[i].state.velocity = 3.0;
            for(int j = 0; j < 15; j++) objectList.dynamic_object_list[i].state.covariance[j] = rand() % 1;
        }

        // static_obstacles
        for(int i = 0; i < numberStaticObjects; i++){
            objectList.static_object_list[i].stamp.sec = 1662975852;
            objectList.static_object_list[i].stamp.nanosec = 321584164;
            objectList.static_object_list[i].id = 2;
            objectList.static_object_list[i].x_centroid = 5.0;
            objectList.static_object_list[i].y_centroid = 5.0;
            objectList.static_object_list[i].z_centroid = 1.0;
            objectList.static_object_list[i].width=10.0;
            objectList.static_object_list[i].length=10.0;
            objectList.static_object_list[i].height=2.0;
            objectList.static_object_list[i].category_list.resize(1);
            objectList.static_object_list[i].category_list[0].category = 10;
            objectList.static_object_list[i].category_list[0].category_probability = 90.2;
            for(int j = 0; j < 6; j++) objectList.static_object_list[i].covariance[0] = rand() % 1;
            objectList.static_object_list[i].state.x = 1020.5;
            objectList.static_object_list[i].state.y = -500.2;
            objectList.static_object_list[i].state.z = -10.2;
            objectList.static_object_list[i].state.yaw = 0.2;
            objectList.static_object_list[i].state.course_rate = 0.0;
            objectList.static_object_list[i].state.course_angle = 0.0;
            objectList.static_object_list[i].state.velocity = 0.0;
            for(int j = 0; j < 15; j++) objectList.static_object_list[i].state.covariance[j] = rand() % 1;
        }
    }
};


//////// ACTUAL TESTS ////////

// Test header and object size
TEST(TestSuite,test_case_1) {

    // create object list
    afius_msgs::msg::ObjectList objectList;

    // create predicted object list
    afius_msgs::msg::PredictedObjectList predictedObjectList;

    // create mock object
    MockObjectList MockObject;

    // fill object list with mocked data
    objectList = MockObject.getObjectList();

    // create shared pointer from object list
    auto objectListPtr = std::make_shared<afius_msgs::msg::ObjectList>(objectList);

    // create prediction object
    Prediction::CV predCV;

    // set timestpes and number of predicted timesteps
    size_t numTimesteps = 10;
    double timestep = 0.05;
    predCV.setTimestep(timestep);
    predCV.setNumTimesteps(numTimesteps);

    // make prediction_cv
    predCV.predict(objectListPtr, predictedObjectList);

    // expect same header values
    EXPECT_TRUE(objectList.header==predictedObjectList.header) << "[Test failed]: Header of object list and predicted list must be the same";
    // expect same object size
    EXPECT_TRUE(objectList.dynamic_object_list.size()==predictedObjectList.predicted_object_list.size()) << "[Test failed]: Size from dynamic_object_list and predicted_object_list must be the same";
    // expect different object size
    EXPECT_FALSE(objectList.static_object_list.size()==predictedObjectList.predicted_object_list.size()) << "[Test failed]: Size from static_object_list and predicted_object_list must be different";
}


// Test if object of dynamic object list is equal predicted_object_list.predicted_object_list.objct
TEST(TestSuite,test_case_2) {

    // create object list
    afius_msgs::msg::ObjectList objectList;

    // create predicted object list
    afius_msgs::msg::PredictedObjectList predictedObjectList;

    // create mock object
    MockObjectList MockObject;

    // fill object list with mocked data
    objectList = MockObject.getObjectList();

    // create shared pointer from object list
    auto objectListPtr = std::make_shared<afius_msgs::msg::ObjectList>(objectList);

    // create prediction object
    Prediction::CV predCV;

    // set timestpes and number of predicted timesteps
    size_t numTimesteps = 10;
    double timestep = 0.05;
    predCV.setTimestep(timestep);
    predCV.setNumTimesteps(numTimesteps);

    // make prediction_cv
    predCV.predict(objectListPtr, predictedObjectList);

    // expect same object values
    EXPECT_TRUE(objectList.dynamic_object_list[0] == predictedObjectList.predicted_object_list[0].object) << "[Test failed]: Object values of object list and predicted list must be the same";
    // expect different object values
    EXPECT_FALSE(objectList.static_object_list[0] == predictedObjectList.predicted_object_list[0].object) << "[Test failed]: Object values of object list and predicted list must be different";
}



// test prediction_cv for static dynamical object
TEST(TestSuite,test_case_3) {

    // create object list
    afius_msgs::msg::ObjectList objectList;

    // create predicted object list
    afius_msgs::msg::PredictedObjectList predictedObjectList;

    // create mock object
    MockObjectList MockObject;
    // modify paras
    double stateX = 10.0;
    double stateY = 10.0;
    double velocity = 0.0;
    double yaw = 0.8;
    MockObject.setPara(stateX, stateY, velocity, yaw);

    // fill object list with mocked data
    objectList = MockObject.getObjectList();

    // create shared pointer from object list
    auto objectListPtr = std::make_shared<afius_msgs::msg::ObjectList>(objectList);

    // create prediction object
    Prediction::CV predCV;

    // set timestpes and number of predicted timesteps
    size_t numTimesteps = 10;
    double timestep = 0.05;
    predCV.setTimestep(timestep);
    predCV.setNumTimesteps(numTimesteps);

    // make prediction_cv
    predCV.predict(objectListPtr, predictedObjectList);

    // expect first x pos equal state_x
    EXPECT_TRUE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].x
    == stateX)
    << "[Test failed]: object x state must be static -> velocity 0: state_list.x = " <<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].x
    << " state_x = "<< stateX;

    // expect first x pos equal state_y
    EXPECT_TRUE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].y
    == stateY)
    << "[Test failed]: object y state must be static -> velocity 0: state_list.y = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].y
    << " state_y = "<< stateY;

    // expect first x pos equal last x pos
    EXPECT_TRUE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].x
    == predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].x)
    << "[Test failed]: object x state must be static -> velocity 0: first state_list.x = " <<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].x
    << " last state_list.x = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].x;

    // expect first y pos equal last y pos
    EXPECT_TRUE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].y
    == predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].y)
    << "[Test failed]: object y state must be static -> velocity 0: first state_list.y = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].y
    << " last state_list.y = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].y;
}


// test prediction_cv for moving dynamical object
TEST(TestSuite,test_case_4) {

    // create object list
    afius_msgs::msg::ObjectList objectList;

    // create predicted object list
    afius_msgs::msg::PredictedObjectList predictedObjectList;

    // create mock object
    MockObjectList MockObject;
    // modify paras
    double stateX = 10.0;
    double stateY = 10.0;
    double velocity = 1.0;
    double yaw = 0.5;
    MockObject.setPara(stateX, stateY, velocity, yaw);

    // fill object list with mocked data
    objectList = MockObject.getObjectList();

    // create shared pointer from object list
    auto objectListPtr = std::make_shared<afius_msgs::msg::ObjectList>(objectList);

    // create prediction object
    Prediction::CV predCV;

    // set timestpes and number of predicted timesteps
    size_t numTimesteps = 60;
    double timestep = 0.05;
    predCV.setTimestep(timestep);
    predCV.setNumTimesteps(numTimesteps);

    // make prediction_cv
    predCV.predict(objectListPtr, predictedObjectList);

    // expect first x pos not equal state_x
    EXPECT_FALSE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].x
    == stateX)
    << "[Test failed]: first x pos not equal state_x: state_list.x = " <<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].x
    << " state_x = "<< stateY;

    // expect first x pos not equal state_y
    EXPECT_FALSE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].y
    == stateY)
    << "[Test failed]: first y pos not equal state_y: state_list.y = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].y
    << " state_y = "<< stateY;

    // expect first x pos not equal last x pos
    EXPECT_FALSE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].x
    == predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].x)
    << "[Test failed]: object x state must be static -> velocity 0: first state_list.x = " <<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].x
    << " last state_list.x = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].x;

    // expect first y pos not equal last y pos
    EXPECT_FALSE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].y
    == predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].y)
    << "[Test failed]: object y state must be static -> velocity 0: first state_list.y = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[0].y
    << " last state_list.y = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].y;


    // expect last x pos =
    double lastX = 12.632747685671113;
    EXPECT_TRUE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].x == lastX)
    << "[Test failed]: object x state must be last_pos_x: last state_list.x = " <<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].x
    << " last_pos_x = "<< lastX;

    // expect last y pos =
    double lastY = 11.43827661581259;
    EXPECT_TRUE(predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].y == lastY)
    << "[Test failed]: object y state must be last_pos_y: last state_list.y = "<<
    predictedObjectList.predicted_object_list[0].object_trajectory_list[0].state_list[numTimesteps - 1].y
    << " last_pos_y = "<< lastY;
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}