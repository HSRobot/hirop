#include "vision/detector.h"
#include "utils/idebug.h"

#include <vision/configure.h>

using namespace hirop_vision;

Detector::Detector(){
    detectionThr = NULL;
    listener = NULL;
    cppLoader = new CppLoader();
    pyLoader = PyLoader::getPyLoader();
    detectorPtr = NULL;

    /**
     *  获取物体训练数据的存放位置
     */
    objectDataPath = getObjectDataPath();
}

int Detector::detectionOnce(const cv::Mat &depthImg, const cv::Mat &colorImg){
    /**
     * @todo
     *      1, 根据训练器名称，加载训练器
     *      2, 根据物体名称，调用训练器加载函数，加载相关数据
     *      3, 建立线程，调用训练器的检测函数
     *
     **/

    if(detectionThr){
        std::cerr << "start detection error: detection thread was runnig" << std::endl;
        return -1;
    }

    if(!detectorPtr){
        std::cerr << "start detection error: load detector was NULL" << std::endl;
        return -1;
    }

    detectorPtr->setColorImg(colorImg);
    detectorPtr->setDepthImg(depthImg);

    /**
     * 暂时将线程功能关闭，因为当前Linemod识别器在线程下运行时有BUG
     */
    __detection(false);
    //    boost::function0<int> f =  boost::bind(&Detector::__detection,this, false);
    //    detectionThr = new boost::thread(f);

    //    // 启动线程
    //    detectionThr->timed_join(boost::posix_time::microseconds(1));

    return 0;

}

int Detector::detection(std::string objectName, std::string detectorName, const cv::Mat &depthImg, const cv::Mat &colorImg){

    if(detectionThr){
        std::cerr << "start detection error: detection thread was runnig" << std::endl;
        return -1;
    }

    if(!detectorPtr){
        std::cerr << "start detection error: load detector was NULL" << std::endl;
        return -1;
    }

    boost::function0<int> f =  boost::bind(&Detector::__detection,this,  true);
    detectionThr = new boost::thread(f);

    // 启动线程
    detectionThr->timed_join(boost::posix_time::microseconds(1));

    return 0;
}

int Detector::setOnStateChangeCallback(DetectStateListener *listener){
    if(listener == NULL){
        std::cerr << "set state listener was error: listener can't be NULL" << std::endl;
        return -1;
    }

    this->listener = listener;
    return 0;
}

int Detector::__detection(bool loop){

    std::vector<pose> results;
    cv::Mat preImg;
    int ret;

    // 开始识别[阻塞式函数]
    ret = detectorPtr->detection();
    if(ret){
        std::cerr << "detection error" << std::endl;
        goto _detectionFaile;
    }

    /**
     *  获取预览图片
     */
    if(detectorPtr->havePreImg()){
        ret = detectorPtr->getPreImg(preImg);
        if(ret){
            std::cerr << "get preview image error" << std::endl;
            goto _detectionFaile;
        }
    }

    /**
     *  获取识别结果
     */
    ret = detectorPtr->getResult(results);
    if(ret){
        std::cerr << "get result error" << std::endl;
        goto _detectionFaile;
    }

    /**
      * @todo 传递给监听者正确的识别结束状态码
      */
    if(listener != NULL)
        listener->onDetectDone(detectorName, ret, results, preImg);

    delete detectionThr;
    detectionThr = NULL;
    return 0;

_detectionFaile:
    delete detectionThr;
    detectionThr = NULL;
    return -1;
}

int Detector::setDetector(const std::string &name, const std::string &objectName, ENTITY_TYPE type, \
                          std::string configFile){

    int ret = 0;
    this->detectorName = name;
    this->objectName = objectName;

    if( cppDetectors.count(name) )
        this->detectorPtr = cppDetectors.at(name);
    else if(pyDetectors.count(name))
        this->detectorPtr = pyDetectors.at(name);
    else if(cppSingleDetectors.count(name) && cppSingleDetectors.at(name).count(objectName))
        this->detectorPtr = cppSingleDetectors.at(name).at(objectName);
    else if(pySingleDetectors.count(name) && pySingleDetectors.at(name).count(objectName))
        this->detectorPtr = pySingleDetectors.at(name).at(objectName);
    else
        this->detectorPtr = NULL;

    if(detectorPtr != NULL)
        return 0;



    if(type == PYTHON)
        this->detectorPtr = pyLoader->loadDetector(name);
    else if(type == CPP)
        this->detectorPtr = cppLoader->loadDetector(name);

    if(detectorPtr == NULL){
        IErrorPrint("detectorPtr was NULL");
        return -1;
    }

    if( !configFile.empty()){
        std::cout << "configFile: "<<configFile<<std::endl;
        YAML::Node privateParam;
        Configure *config = new Configure(configFile);
        if(!config->getPrivateParams(privateParam)){
            if(this->detectorPtr->parseConfig(privateParam) !=0 ){
                IErrorPrint("Get parseConfig  NULL");
                return -1;
            }
        }else{
            IErrorPrint("Get PrivateParam  NULL");
        }
    }

    std::string prefix = objectDataPath;
    std::string detectorName = name;

    ret = detectorPtr->loadData(prefix + detectorName + "/" + objectName, objectName);
    if(ret){
        IErrorPrint("detctorPtr loadData failed");
        return -1;
    }

    // 保存相关的检测器实例
    if(type == PYTHON){
        if(detectorPtr->isMultiDetector())
            pyDetectors.insert(std::pair<std::string, IDetector *>(name, this->detectorPtr));
        else{
            if(pySingleDetectors.count(name))
                pySingleDetectors.at(name).insert(std::pair<std::string, IDetector *>(objectName, this->detectorPtr));
            else{
                std::map<std::string, IDetector*> tmp;
                tmp.insert(std::pair<std::string, IDetector *>(objectName, this->detectorPtr));
                pySingleDetectors.insert(std::make_pair(name, tmp));
            }
        }
    }else{
        if(detectorPtr->isMultiDetector())
            cppDetectors.insert(std::pair<std::string, IDetector *>(name, this->detectorPtr));
        else{
            if(cppSingleDetectors.count(name))
                cppSingleDetectors.at(name).insert(std::pair<std::string, IDetector *>(objectName, this->detectorPtr));
            else{
                std::map<std::string, IDetector*> tmp;
                tmp.insert(std::pair<std::string, IDetector *>(objectName, this->detectorPtr));
                cppSingleDetectors.insert(std::make_pair(name, tmp));
            }
        }
    }
    return 0;
}

void Detector::getDetectorList(std::vector<std::string> &detectorList){
    cppLoader->getDetectorList(detectorList);
    pyLoader->getDetectorList(detectorList);
}

int Detector::getObjectList(std::string detecorName, std::vector<std::string> &objectList){

    boost::filesystem::path objectDataDir(objectDataPath + detecorName);
    boost::filesystem::directory_iterator end;

    for(boost::filesystem::directory_iterator pos(objectDataDir); pos != end; ++pos){
        if(boost::filesystem::is_directory(*pos)){
            objectList.push_back(pos->path().filename().string());
        }
    }

    return objectList.size();

}
