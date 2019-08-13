#pragma once

#include <iostream>
 
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"


#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;



using namespace tensorflow;
using namespace std;



class TensorflowCCVisoin{
public:
    TensorflowCCVisoin();
    ~TensorflowCCVisoin();
    /**
     * 外部调用接口
     */
public:
    /**
     * init 初始化
     */
    int init(int img_width, int img_height);
    int LoadModel(const string& model_path);
    int detection(cv::Mat tempImage, vector<float> &pos);    
protected:
    /**
     * 功能描述：初始化 tensorflow的网络 会话参数
     * 输入参数： 无
     * 返回参数： int 0 为初始化正常，-1的话为初始化失败
     */
    int InitParam( int img_width, int img_height);
    /**
     * 功能描述：载入网络模型的pb文件 
     * 输入参数：string pb文件的路径， pbtxt标签的路径
     * 返回参数：int 0为载入模型正常 ，-1为失败
     */
    int LoadPbParam();
    /**
     * 功能描述：在网络中输入图像
     * 输入参数：图像的指针和图像的格式
     * 返回参数：int 0 是否记载成功 其他即为错误码
     */
    int LoadImg(cv::Mat tempImage);

    /**
     * 功能描述：在网络中，对输入图像进行推理
     * 输入参数：推理的结果和 推理的archor的定位坐标 和矩形框
     * 返回参数：int 0 是否推理成功 其他即为错误码
     */
    int TfDetection();

    /**
     * 功能描述：在网络中，获取推理的结果
     * 输入参数：float 类型的数组
     * 返回参数：int 0 是否返回成功数组
     */
    int TfGetResult(std::vector<float> &result);
private:
    string input_tensor_name; //输入 tensor变量的类型 
    string model_path;
    string image_path;
    Session* session; //会话 句柄
    GraphDef graphdef; //Graph Definition for current model  张量图
    vector<tensorflow::Tensor> outputs; //推理输出结果
    std::vector<string> out_put_nodes;  //注意，在object detection中输出的三个节点名称为以下三个
    int input_width, input_height;

    cv::Mat tempMat; //后面去掉
    Tensor resized_tensor;
private:
   int CVMat_to_Tensor(Mat img,Tensor* output_tensor,int input_rows,int input_cols);

};

