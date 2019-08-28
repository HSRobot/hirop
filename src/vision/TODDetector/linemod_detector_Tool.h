#ifndef __LINEMOD_DETECTOR_H__
#define __LINEMOD_DETECTOR_H__

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
#else
#include <opencv2/rgbd/rgbd.hpp>
#endif
#include <opencv2/rgbd/linemod.hpp>
// #include "vision/c_base_detector.h"
#include "object_recognition_renderer/renderer3d.h"
#include <object_recognition_renderer/utils.h>
#include "vision.h"
// #include "hplugin/hpluginloader.h"
#include "yaml-cpp/yaml.h"
using namespace hirop_vision;

class LinemodDetector{

    struct ObjData{
      ObjData(
              std::vector<cv::Vec3f> _pts_ref,
              std::vector<cv::Vec3f> _pts_model,
              std::string _match_class,
              const float _match_sim,
              const float _icp_dist,
              const float _icp_px_match,
              const cv::Matx33f _r,
              const cv::Vec3f _t){
        pts_ref = _pts_ref;
        pts_model = _pts_model;
        match_class = _match_class;
        match_sim = _match_sim;
        icp_dist = _icp_dist;
        icp_px_match = _icp_px_match,
        r = _r;
        t = _t;
        check_done = false;
      }
      std::vector<cv::Vec3f> pts_ref;
      std::vector<cv::Vec3f> pts_model;
      std::string match_class;
      float match_sim;
      float icp_dist;
      float icp_px_match;
      cv::Matx33f r;
      cv::Vec3f t;
      bool check_done;
    };

public:
    /**
     * @brief   构造函数
     */
    LinemodDetector();

    /**
     * @brief   析构函数
     */
    ~LinemodDetector();

    /**
     * @brief   实现具体的识别功能
     * @return  0 成功 -1 失败
     */
    int detection();

    /**
     * @brief   加载相关的训练结果
     * @param   [objectName] 需要识别的物体
     * @return  0 成功 -1 失败
     */
    int loadData(const std::string path, const std::string &objectName, const std::string &type);

    /**
     * @brief   获取识别的结果
     * @param[out] poses， 保存识别的结果
     * @return  0 成功 -1 失败
     */
    int getResult(std::vector<pose> &poses);

    /**
     * @brief   向检测器传递图像数据
     * @param   [inputImg] 输入，传递的图像
     * @return  void
     */
    void setDepthImg(const cv::Mat &inputImg);

    /**
     * @brief   向检测器传递图像数据
     * @param   [inputImg] 输入，传递的图像
     * @return  void
     */
    void setColorImg(const cv::Mat &inputImg);

    int readYamlData(const YAML::Node &node);

private:

    /**
     * @brief   初始化默认参数
     */
    void initParam();

    /**
     * @brief   从文件中加载Detctor
     */
    void loadDetctor(const std::string file_name, cv::linemod::Detector *value);

    /**
     * @brief   从文本文件中加载vector<Mat>
     * @param[in]   fileName:文本文件名称
     * @param[out]  out:将文本中的内容加载至out中
     * @param[in]   rows:Mat的行数
     * @param[in]   cols:Mat的列数
     * @return  void
     */
    void loadVectorMat(std::string &fileName, std::vector<cv::Mat> &out, int rows, int cols);

    /**
     * @brief       icp处理，将参考点云和目标点云进行配准 pts_ref = pts_model * R + T
     *
     * 每次求出一个将ref点云对齐到model点云的旋转平移矩阵，然后根据得到的矩阵对model点云进行变换
     * 并且将R T进行更新。每次的R T都是model点云对应的变换矩阵
     *
     * @param[in] pts_ref   参考点云，通过传感器获取的深度图转换而来
     * @param[in] pts_model 目标点云，通过渲染得到的深度图转换而来
     * @param[in,out] R     旋转矩阵，输出配准后的旋转矩阵
     * @param[in,out] T     平移矩阵，输出配准后的平移矩阵
     * @param[out] px_inliers_ratio 两个点云具有相同深度的像素数
     * @param[in] mode      计算模式，1-粗定位（较少的迭代次数） 2-最优解（较多的迭代次数）
     * @return  内点数相对与总点数的比值
     */
    float icpCloudToCloud(const std::vector<cv::Vec3f> &pts_ref,\
                          std::vector<cv::Vec3f> &pts_model, \
                          cv::Matx33f& R, cv::Vec3f& T, float &px_inliers_ratio, int mode);
    /**
     * @brief       获取两个点云的欧式距离
     * @param
     * @return
     */
    float getL2distClouds(const std::vector<cv::Vec3f> &model,\
                          const std::vector<cv::Vec3f> &ref, float &dist_mean, const float mode=0);
    /**
     * @brief
     * @param
     * @return
     */
    void transformPoints(const std::vector<cv::Vec3f> &src,\
                         std::vector<cv::Vec3f>& dst, const cv::Matx33f &R, const cv::Vec3f &T);

    /**
     * @brief
     * @param
     * @return
     */
    void getMean(const std::vector<cv::Vec3f> &pts, cv::Vec3f& centroid);

    /**
     * @brief
     * @param
     * @return
     */
    float matToVec(const cv::Mat_<cv::Vec3f> &src_ref, \
                   const cv::Mat_<cv::Vec3f> &src_mod, std::vector<cv::Vec3f>& pts_ref, \
                   std::vector<cv::Vec3f>& pts_mod);

    /**
     * @brief 
     */
    void RT2Pose(const cv::Matx33f &R, const cv::Vec3f &T, pose &pose);


private:

    /**
     * @brief Linemod 相关的渲染参数
     */
    int param_n_points_;
    int param_angle_step_;
    double param_radius_min_;
    double param_radius_max_;
    double param_radius_step_;
    int param_width_;
    int param_height_;
    double param_near_;
    double param_far_;
    double param_focal_length_x_;
    double param_focal_length_y_;


    float th_obj_dist_;
    float verbose_;

    std::string meshPath;

    // opencv linemod对象指针
    cv::linemod::Detector *detector_;

    // 保存输入的图像数据
    cv::Mat depth_;
    cv::Mat color_;

    std::map <std::string, std::vector<cv::Mat> > Rs_;
    std::map <std::string, std::vector<cv::Mat> > Ts_;
    std::map <std::string, std::vector<cv::Mat> > Ks_;
    std::map <std::string, std::vector<float> > distances_;

    std::map <std::string, RendererIterator*> renderer_iterator_s;

    Renderer3d *renderer_;
    RendererIterator *renderer_iterator_;

    float icp_dist_min_;
    float px_match_min_;

    std::vector<ObjData> objs_;

    // 保存识别成功后的位姿，由于可能在一张图片识别到多个一样的物体，所以采用vector
    std::vector<pose> poses;

    cv::Mat tmpMat_;
private:

};

// H_DECLARE_PLUGIN(IDetector)

#endif
