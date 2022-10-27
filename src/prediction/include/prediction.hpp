/******************************************************************************
 *  @file       prediction.hpp
 *
 *  <!-- Start of section for manual description -->
 *  @brief      Trajectory Prediction, constant velocity or ai based
 *  <!-- End of section for manual description -->
 *
 *  @author     Stefan Sass <stefan.sass@ovgu.de>
 *  @author

 ******************************************************************************/
#pragma once

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "model.hpp"
#include "afius_msgs/msg/object_list.hpp"
#include "afius_msgs/msg/ego_state.hpp"
#include "afius_msgs/msg/polygon_list.hpp"
#include "afius_msgs/msg/predicted_object_list.hpp"

// CONSTANTS
# define TIMESTEPS 150
# define AGENTS 5
# define BATCH_SIZE 64
# define HISTORY 10
# define ROW 100
# define COLUMN 100
# define CHANNEL 1
# define INPUT_DIM_TRA 2
# define INPUT_DIM_Z 10
# define OUTPUT_DIM 2


namespace Prediction {

    /// Prediction constant velocity
    /// input message afius_msgs::msg::ObjectList
    /// output message afius_msgs::msg::PredictedObjectList
    class CV{
        private:
            size_t numTimesteps;
            double timestep;
            double posX;
            double posY;

        public:
        /// Constructor
        CV(){};

        /// Constructor with parans
        /// @param numTs
        /// @param ts
        CV(size_t numTs, double ts){
            numTimesteps = numTs;
            timestep = ts;
            }

        /// Deconstructor
        ~CV(){};

        /// setTimestep method for set the timestep attribute
        /// @brief set timestep: distance between discrete time steps in seconds
        /// @param ts
        void setTimestep(double ts){timestep = ts;}

        /// setNumTimesteps method for set the numTimesteps attribute
        /// @brief set numTimesteps: number of discrete time steps to predict
        /// @param numTs
        void setNumTimesteps(size_t numTs){numTimesteps = numTs;}

        /// method for prediction constant velocity
        /// @brief constant velocity preciction
        /// @param objectList object list shared pointer
        /// @param predictionObjectList predicted list with constant velocity trajectory
        void predict(const afius_msgs::msg::ObjectList::SharedPtr objectList, afius_msgs::msg::PredictedObjectList &predictionObjectList){

            // set header
            predictionObjectList.header = objectList->header;

            // resize prediction object list
            predictionObjectList.predicted_object_list.resize(objectList->dynamic_object_list.size());

            for (size_t object_idx = 0; object_idx < objectList->dynamic_object_list.size(); ++object_idx){

                // set object
                predictionObjectList.predicted_object_list[object_idx].object =
                        objectList->dynamic_object_list[object_idx];

                // resize trajectory list only one trajectory per object -> set to 1
                predictionObjectList.predicted_object_list[object_idx].object_trajectory_list.resize(1);

                // set Duration
                predictionObjectList.predicted_object_list[object_idx].object_trajectory_list[0].delta_t =
                        rclcpp::Duration(0, timestep*1e9);

                // resize trajectory
                predictionObjectList.predicted_object_list[object_idx].object_trajectory_list[0].state_list.resize(numTimesteps);

                // set start value
                posX = objectList->dynamic_object_list[object_idx].state.x;
                posY = objectList->dynamic_object_list[object_idx].state.y;

                // fill object state to trajectory and compute new pos of x, y for each object state
                for (size_t time_idx = 0; time_idx < numTimesteps ; ++time_idx){
                    // set state
                    predictionObjectList.predicted_object_list[object_idx].object_trajectory_list[0].state_list[time_idx] =
                            objectList->dynamic_object_list[object_idx].state;

                    // calculate x,y
                    posX +=  objectList->dynamic_object_list[object_idx].state.velocity * timestep *
                                cos(objectList->dynamic_object_list[object_idx].state.yaw);

                    posY += objectList->dynamic_object_list[object_idx].state.velocity * timestep *
                               sin(objectList->dynamic_object_list[object_idx].state.yaw);

                    // set x pos
                    predictionObjectList.predicted_object_list[object_idx].object_trajectory_list[0].state_list[time_idx].x = posX;

                    // set y pos
                    predictionObjectList.predicted_object_list[object_idx].object_trajectory_list[0].state_list[time_idx].y = posY;
                }
            }
        }
    };




    /// Prediction NN
    /// input message afius_msgs::msg::ObjectList
    /// input message afius_msgs::msg::EgoState
    /// input message afius_msgs::msg::PolygonList
    /// output message afius_msgs::msg::PredictedObjectList
    class NN{

    public:
        // Constructor
        NN():
        // initialize
                inputMap(BATCH_SIZE*AGENTS*HISTORY*ROW*COLUMN*CHANNEL, 1.0),
                inputTra(BATCH_SIZE*AGENTS*HISTORY*INPUT_DIM_TRA, 1.0),
                inputZ(BATCH_SIZE*AGENTS*INPUT_DIM_Z, 1.0)
        {}

        // Deconstructor
        ~NN(){}


        // init Model without gpu settings
        void onInit(const char *path)
        {
            // init without gpu settings
            model.onInit(path, inputNameVec, inputDimVec, inputLengthVec);

        }

        // init Model with gpu settings
        void onInit(const char *path, int gpuID, double gpuMemFrac)
        {
            // init with gpu settings
            model.onInit(path, inputNameVec, inputDimVec, inputLengthVec, gpuID, gpuMemFrac);
        }



        void processData(afius_msgs::msg::ObjectList & objectList,
                         afius_msgs::msg::EgoState & egoState,
                         afius_msgs::msg::PolygonList & polygonList,
                         afius_msgs::msg::PredictedObjectList & predictedList){



            // TODO preprocess input_vals_map, input_vals_tra, input_vals_z
            //inputMap[0] = 1.0;
            //inputTra[0] = 1.0;
            //inputZ[0] = 1.0;

            // set input tensor
            model.updateInputTensor(inputVec);

            // predict
            model.runSession(predictedTrajectory);

            // TODO post process prediction
            //predictedList[0] = 1.0;
        }

        private:

            // model para
            const char *modelPath;
            std::vector<std::string> inputNameVec = {"map", "tra", "z"};
            std::vector<std::vector<int64_t>> inputDimVec = {{BATCH_SIZE * AGENTS, HISTORY, ROW, COLUMN, CHANNEL},
                                                             {BATCH_SIZE * AGENTS, HISTORY, INPUT_DIM_TRA},
                                                             {BATCH_SIZE * AGENTS, INPUT_DIM_Z}};

            std::vector<std::size_t> inputLengthVec = {{(BATCH_SIZE * AGENTS * HISTORY * ROW * COLUMN * CHANNEL) * sizeof(float)},
                                                    {(BATCH_SIZE * AGENTS * HISTORY * INPUT_DIM_TRA) * sizeof(float)},
                                                    {(BATCH_SIZE * AGENTS * INPUT_DIM_Z) * sizeof(float)}};


            std::vector<std::vector<float>> inputTensor;
            std::vector<float> predictedTensor;

            // load and initialize model
            Model model;

            // in-/ouput tensor
            std::vector<float> inputMap;
            std::vector<float> inputTra;
            std::vector<float> inputZ;
            std::vector<float> predictedTrajectory;

            // vector with pointers of model inputs
            std::vector<std::vector<float> * > inputVec = {&inputMap, &inputTra, &inputZ};
    };
} // Prediction