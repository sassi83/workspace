/******************************************************************************
 *  @file       model.hpp
 *
 *  <!-- Start of section for manual description -->
 *  @brief      lib for load and use NN models (tensorflow c api)
 *  <!-- End of section for manual description -->
 *
 *  @author     Stefan Sass <stefan.sass@ovgu.de>
 *  @author

 ******************************************************************************/
#ifndef MODEL_HPP
#define MODEL_HPP

// standard libs
#include <iostream>
#include <vector>
#include <cstring>
#include <fstream>

// tensorflow
#include <tensorflow/c/c_api.h>



class Model {

private:

    TF_Buffer * buffer;
    TF_Session * sess;
    TF_SessionOptions * sessOptions;
    TF_Graph * graph;
    TF_Status * status;

    // input para for a set of inputs
    std::vector<TF_Output> inputOp;
    std::vector<std::vector<float>> inputValues;
    std::vector<TF_Tensor *> inputTensor;
    std::vector < std::vector < std::int64_t >> inputDims;

    // output para for one output
    TF_Tensor * outputTensor = nullptr;
    TF_Output outputOp;
    std::vector<std::size_t> inputLength;

    // private methods
    static void DeallocateBuffer(void *data, size_t) {
        std::free(data);
    }

    static TF_Buffer *ReadBufferFromFile(const char *file) {
        std::ifstream f(file, std::ios::binary);
        if (f.fail() || !f.is_open()) { return nullptr; }
        if (f.seekg(0, std::ios::end).fail()) { return nullptr; }
        auto fsize = f.tellg();
        if (f.seekg(0, std::ios::beg).fail()) { return nullptr; }
        if (fsize <= 0) { return nullptr; }
        auto data = static_cast<char *>(std::malloc(fsize));
        if (f.read(data, fsize).fail()) { return nullptr; }

        auto buf = TF_NewBuffer();
        buf->data = data;
        buf->length = fsize;
        buf->data_deallocator = DeallocateBuffer;
        return buf;
    }

    void loadGraph(const char *graph_path) {
        buffer = ReadBufferFromFile(graph_path);
        status = TF_NewStatus();
        auto options = TF_NewImportGraphDefOptions();
        graph = TF_NewGraph();
        TF_GraphImportGraphDef(graph, buffer, options, status);
        if (graph == nullptr) {
            std::cout << "Can't load graph" << std::endl;

        } else {
            std::cout << "loaded graph"<< std::endl;
        }

        // delete options, buffer
        TF_DeleteImportGraphDefOptions(options);
        TF_DeleteBuffer(buffer);
    }

    // method for set graph operation
    // std::vector<std::string> inputName
    // inputName = {"map", "tra", "z"}
    void setGraphOperation(std::vector<std::string> inputName) {
        for(size_t i = 0; i < inputName.size(); i++) {
            inputOp.push_back(TF_Output{TF_GraphOperationByName(graph, inputName[i].c_str()), 0});
            if (inputOp[i].oper == nullptr) {
                std::cout << "Can't init input_op"<< std::endl;
            } else {
                std::cout << "init input_op"<< std::endl;
            }
        }
    }


    void createInputTensor(std::vector<std::vector<int64_t>> inputDimVec, std::vector<std::size_t> inputLen) {

        for(size_t i=0; i < inputDimVec.size(); i++) {

            // set input dim
            // std::vector < std::vector < std::int64_t >> inputDims
            // inputDims = {input_dims_map, input_dims_tra}, input_dims_z}
            // const std::vector <std::int64_t> input_dims_map = {BATCH_SIZE * AGENTS, ROW, COLUMN, CHANNEL};
            // const std::vector <std::int64_t> input_dims_tra = {BATCH_SIZE * AGENTS, HISTORY * INPUT_DIM_TRA};
            // const std::vector <std::int64_t> input_dims_z = {BATCH_SIZE * AGENTS, INPUT_DIM_Z};
            inputDims.push_back(inputDimVec[i]);
            // set input length
            // std::vector<std::size_t> inputLength;
            // inputLength = {len_map, len_tra, len_z}
            // std::size_t len_map = (BATCH_SIZE * AGENTS * ROW * COLUMN * CHANNEL) * sizeof(float);
            // std::size_t len_tra = (BATCH_SIZE * AGENTS * HISTORY * INPUT_DIM_TRA) * sizeof(float);
            // std::size_t len_z = (BATCH_SIZE * AGENTS * INPUT_DIM_Z) * sizeof(float);
            inputLength.push_back(inputLen[i]);


            // set iput values to 0.0
            std::vector<float> zeroVector(inputLen[i], 0.0);
            inputValues.push_back(zeroVector);

            // create input tensor
            inputTensor.push_back(TF_AllocateTensor(TF_FLOAT, inputDims[i].data(), inputDims[i].size(), inputLength[i]));
            inputLength[i] = std::min(inputLength[i], TF_TensorByteSize(inputTensor[i]));

            if (inputLength[i] != 0) {
                memcpy(TF_TensorData(inputTensor[i]), inputValues[i].data(), inputLength[i]);
                // print dimension and shape
                std::cout << "input tensor "<< i <<" created: with dimension = " << inputDimVec[i].size()<< " and shape (";
                for (int64_t shape: inputDimVec[i]) std::cout << shape << " ";
                std::cout << ")" << std::endl;

            } else {
                std::cout << "input tensor "<< i <<" error "<< std::endl;
                TF_DeleteTensor(inputTensor[i]);
            }
        }
    }

    void createOutputTensor(void) {
        outputOp = TF_Output{TF_GraphOperationByName(graph, "Identity"), 0};
        if (outputOp.oper == nullptr) {
            std::cout << "Can't init outputOp"<< std::endl;
        } else {
            std::cout << "init outputOp"<< std::endl;
        }
    }


    TF_SessionOptions * createSessionOptions(int gpuId, double gpuMemFrac)
    {
        TF_Status * stat = TF_NewStatus();
        TF_SessionOptions * opt = TF_NewSessionOptions();

        // default set GPU 0
        std::vector <uint8_t> config = {0x32, 0xe, 0x9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0x3f, 0x20, 0x1, 0x2a, 0x1, 0x30, 0x38, 0x1};

        // convert the desired percentage into a byte-array
        uint8_t *bytes = reinterpret_cast<uint8_t *>(&gpuMemFrac);

        // put it to the config byte-array, from 3 to 10:
        for (size_t i = 0; i < sizeof(gpuMemFrac); i++) {
            config[i + 3] = bytes[i];
        }

        // set GPU 1-3 default GPU 0
        if (gpuId == 1) config[15] = 0x31;
        if (gpuId == 2) config[15] = 0x32;
        if (gpuId == 3) config[15] = 0x33;

        TF_SetConfig(opt, config.data(), config.size(), stat);

        if (TF_GetCode(stat) != TF_OK) {
            std::cout << "Can't set options: " << TF_Message(stat) << std::endl;
            TF_DeleteStatus(stat);
            return nullptr;
        }

        TF_DeleteStatus(stat);

        return opt;
    }

    void createSession(void) {
        sessOptions = TF_NewSessionOptions();
        sess = TF_NewSession(graph, sessOptions, status);
        if (TF_GetCode(status) != TF_OK) {
            std::cout << "Can't create new session"<< std::endl;
        } else {
            std::cout << "new session created on gpu 1"<< std::endl;
        }
    }


    void createSessionWithOptions(int gpuId, double gpuMemFrac) {
        // start with Gpu $(gpuId)
        sessOptions = createSessionOptions(gpuId, gpuMemFrac);
        sess = TF_NewSession(graph, sessOptions, status);
        if (TF_GetCode(status) != TF_OK) {
            // fallback to gpu 0
            std::cout << "Can't create new session on gpu "<< gpuId
            << " fallback to  gpu 0" << std::endl;
            sessOptions = createSessionOptions(0, gpuMemFrac);
            sess = TF_NewSession(graph, sessOptions, status);
            if (TF_GetCode(status) != TF_OK) {
                std::cout << "Can't create new session on gpu 0"<< std::endl;
            } else {
                std::cout << "new session created on gpu 0"<< std::endl;
            }
        } else {
            std::cout << "new session created on gpu "<< gpuId<< std::endl;
        }
    }


    void closeSession(void) {
        TF_CloseSession(sess, status);
        if (TF_GetCode(status) != TF_OK) {
            std::cout << "Error close session"<< std::endl;
        }
    }


    void deleteSession(void) {
        TF_DeleteSession(sess, status);
        if (TF_GetCode(status) != TF_OK) {
            std::cout << "Error delete session"<< std::endl;
        }
        TF_DeleteSessionOptions(sessOptions);
        TF_DeleteStatus(status);
        TF_DeleteGraph(graph);
    }


    const char *DataTypeToString(TF_DataType data_type) {
        switch (data_type) {
            case TF_FLOAT:
                return "TF_FLOAT";
            case TF_DOUBLE:
                return "TF_DOUBLE";
            case TF_INT32:
                return "TF_INT32";
            case TF_UINT8:
                return "TF_UINT8";
            case TF_INT16:
                return "TF_INT16";
            case TF_INT8:
                return "TF_INT8";
            case TF_STRING:
                return "TF_STRING";
            case TF_COMPLEX64:
                return "TF_COMPLEX64";
            case TF_INT64:
                return "TF_INT64";
            case TF_BOOL:
                return "TF_BOOL";
            case TF_QINT8:
                return "TF_QINT8";
            case TF_QUINT8:
                return "TF_QUINT8";
            case TF_QINT32:
                return "TF_QINT32";
            case TF_BFLOAT16:
                return "TF_BFLOAT16";
            case TF_QINT16:
                return "TF_QINT16";
            case TF_QUINT16:
                return "TF_QUINT16";
            case TF_UINT16:
                return "TF_UINT16";
            case TF_COMPLEX128:
                return "TF_COMPLEX128";
            case TF_HALF:
                return "TF_HALF";
            case TF_RESOURCE:
                return "TF_RESOURCE";
            case TF_VARIANT:
                return "TF_VARIANT";
            case TF_UINT32:
                return "TF_UINT32";
            case TF_UINT64:
                return "TF_UINT64";
            default:
                return "Unknown";
        }
    }

public:

    // Constructor
    Model(){};
    // Deconstructor
    ~Model() {
        closeSession();
        deleteSession();
    }

    // init Model
    void onInit(const char *path, std::vector<std::string> inputName,
              std::vector<std::vector<int64_t>> inputDimVec,
              std::vector<std::size_t> inputLen)
    {

        // load graph
        loadGraph(path);

        // graph op for input tensor
        setGraphOperation(inputName);

        // create session
        createSession();

        // input Tensor
        createInputTensor(inputDimVec, inputLen);

        // output Tensor
        createOutputTensor();
    }

    // init Model
    void onInit(const char *path, std::vector<std::string> inputName,
                std::vector<std::vector<int64_t>> inputDimVec,
                std::vector<std::size_t> inputLen, int gpuId, double gpuMemFrac)
    {

        // load graph
        loadGraph(path);

        // graph op for input tensor
        setGraphOperation(inputName);

        // create session with gpu limitations 0-1 (0-100%) and used gpu 0/1/2/3
        createSessionWithOptions(gpuId, gpuMemFrac);

        // input Tensor
        createInputTensor(inputDimVec, inputLen);

        // output Tensor
        createOutputTensor();
    }


    // update input
    void updateInputTensor(std::vector<std::vector<float> * > inputVec)
    {
        for(size_t i = 0; i < inputVec.size(); i++)
        {
            memcpy(TF_TensorData(inputTensor[i]), inputVec[i]->data(), inputLength[i]);
        }
    }

    void runSession(std::vector<float> & predictedTensor) {
        status = TF_NewStatus();

        TF_SessionRun(sess, nullptr, // Run options.
                      inputOp.data(), inputTensor.data(),
                      3, // Input tensors, input tensor values, number of inputs.
                      &outputOp, &outputTensor, 1, // Output tensors, output tensor values, number of outputs.
                      nullptr, 0, // Target operations, number of targets.
                      nullptr, // Run metadata.
                      status // Output status.
        );

        if (TF_GetCode(status) != TF_OK) {
            std::cout << "Error run session code=" << TF_GetCode(status)<< std::endl;
        }

        // save output values
        auto data = static_cast<float *>(TF_TensorData(outputTensor));
        auto size = TF_TensorByteSize(outputTensor) / TF_DataTypeSize(TF_TensorType(outputTensor));
        predictedTensor = {data, data + size};
    }

};

#endif /*MODEL_HPP*/
