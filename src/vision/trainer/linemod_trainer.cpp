#include "linemod_trainer.h"

#include <sstream>
#include <fstream>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include "object_recognition_renderer/renderer3d.h"
#include <object_recognition_renderer/utils.h>

#define LINEMOD_VIZ_IMG 0

#if LINEMOD_VIZ_IMG
#include <opencv2/highgui/highgui.hpp>
#endif

#include <utils/idebug.h>

LinemodTrainer::LinemodTrainer():CBaseTrainer("Linemod"){
    initParam();
}

void LinemodTrainer::initParam(){
    param_n_points_ = 150;
    param_angle_step_ = 10;
    param_radius_min_ = 0.6;
    param_radius_max_ = 1.1;
    param_radius_step_ = 0.4;
    param_width_ = 960;
    param_height_ = 540;
    param_focal_length_x_ = 525.0;
    param_focal_length_y_ = 525.0;
    param_near_ = 0.1;
    param_far_ = 1000;

    meshPath = "coke.stl";
}

int LinemodTrainer::train(){

    detector_ = cv::linemod::getDefaultLINEMOD();

    // 创建渲染器
    Renderer3d renderer = Renderer3d(meshPath);

    // 设置渲染参数
    renderer.set_parameters(param_width_, param_height_, param_focal_length_x_, \
                            param_focal_length_y_, param_near_, param_far_);

    // 构建渲染迭代器
    RendererIterator renderer_iterator = RendererIterator(&renderer, param_n_points_);

    // 设置迭代器的参数
    renderer_iterator.angle_step_ = param_angle_step_;
    renderer_iterator.radius_min_ = float(param_radius_min_);
    renderer_iterator.radius_max_ = float(param_radius_max_);
    renderer_iterator.radius_step_ = float(param_radius_step_);

    cv::Mat image, depth, mask;
    cv::Matx33d R;
    cv::Vec3d T;
    cv::Matx33f K;

    // 迭代获取渲染出来的图片
    for(size_t i = 0; !renderer_iterator.isDone(); i++, ++renderer_iterator){

        std::stringstream status;
        status << "Loading images " << (i+1) << "/"
               << renderer_iterator.n_templates();
        std::cout << status.str();

        cv::Rect rect;
        renderer_iterator.render(image, depth, mask, rect);

        // 获取旋转矩阵
        R = renderer_iterator.R_obj();

        // 获取平移矩阵
        T = renderer_iterator.T();

        // 获取不知道啥
        float distance = renderer_iterator.D_obj() - float(depth.at<ushort>(depth.rows/2.0f, depth.cols/2.0f)/1000.0f);
        K = cv::Matx33f(float(param_focal_length_x_), 0.0f, \
                        float(rect.width)/2.0f, 0.0f, float(param_focal_length_y_), \
                        float(rect.height)/2.0f, 0.0f, 0.0f, 1.0f);

        std::vector<cv::Mat> sources(2);
        sources[0] = image;
        sources[1] = depth;

#if LINEMOD_VIZ_IMG
        if(!image.empty()){
            cv::imshow("Renderer", image);
            cv::waitKey(1);
        }
#endif

        /**
         *  检测渲染出来的结果是否可用,如果不可用则 continue
         */
        int template_in = detector_->addTemplate(sources, "object1", mask);
        if (template_in == -1){
            for (size_t j = 0; j < status.str().size(); ++j)
                std::cout << '\b';
            continue;
        }

        /**
         *  缓存渲染结果
         */
        Rs_.push_back(cv::Mat(R));
        Ts_.push_back(cv::Mat(T));
        distances_.push_back(distance);
        Ks_.push_back(cv::Mat(K));

        for (size_t j = 0; j < status.str().size(); ++j)
            std::cout << '\b';
    }

    std::cout << "[LinemodTrainer]: \033[32m trianer finish \033[0m" << std::endl;
    return 0;
}

int LinemodTrainer::saveData(std::string path){
    std::cout << "[LinemodTrainer]: saving data to " << path << std::endl;

    std::string RsFileName= "RS.txt";
    std::string TsFileName= "TS.txt";
    std::string KsFileName= "KS.txt";
    std::string DistancesFileName= "distances.txt";
    std::string stlFileName = path+"mesh.stl";

    std::ofstream RsOut(path + RsFileName, std::fstream::out);
    std::ofstream TsOut(path + TsFileName, std::fstream::out);
    std::ofstream KsOut(path + KsFileName, std::fstream::out);
    std::ofstream DsOut(path + DistancesFileName, std::fstream::out);

    if(RsOut && TsOut && KsOut && DsOut){
        for(int i = 0; i < Rs_.size(); i++){
            RsOut << Rs_[i] << "|";
            TsOut << Ts_[i] << "|";
            KsOut << Ks_[i] << "|";
            DsOut << distances_[i] << " ";
        }
        RsOut.close();
        TsOut.close();
        KsOut.close();
        DsOut.close();
        // 序列化opencv detector对象
        saveCVDetector(path);
        boost::filesystem::copy_file(meshPath, stlFileName);
        std::cout << "\033[32m[LinemodTrainer]: saving data success\033[0m" << std::endl;
    }else{
        std::cout << "[LinemodTrainer]: saving data error" << std::endl;
        return -1;
    }

    return 0;
}

int LinemodTrainer::saveCVDetector(const std::string &path){
    std::string detectorFileName = path + "detector.xml";

    cv::FileStorage fs(detectorFileName, cv::FileStorage::WRITE);
    detector_->write(fs);

#if CV_MAJOR_VERSION == 3
    std::vector<cv::String> ids = detector_->classIds();
#else
    std::vector < std::string > ids = detector_->classIds();
#endif
    fs << "classes" << "[";
    for (int i = 0; i < (int) ids.size(); ++i)
    {
        fs << "{";
        detector_->writeClass(ids[i], fs);
        fs << "}"; // current class
    }
    fs << "]"; // classes
    fs.release();
}

int LinemodTrainer::deleteData(){
    return 0;
}

int LinemodTrainer::parseConfig(const YAML::Node &node){

    if(!node["stlFile"]){
        IErrorPrint("Dose not stlFile param");
        return -1;
    }
    meshPath = node["stlFile"].as<std::string>();
    return 0;
}

int LinemodTrainer::feedback(){
    return 0;
}

LinemodTrainer::~LinemodTrainer(){

}

H_EXPORT_PLUGIN(LinemodTrainer, "LinemodTrainer", "1.0")
