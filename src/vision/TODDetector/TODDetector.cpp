#include <TODDetector.h>
#define DEBUG
#include <utils/idebug.h>
#include <string>
template <typename T>
string toString(T& t){
    ostringstream os;
    os << t;
    return os.str();
}

TODDetector::TODDetector():CBaseDetector("TODDetector", false){
    TCCVSION = new TensorflowCCVisoin();
    LineDeTool = new LinemodDetector();
    LineDeTool->loadData("/home/fshs/hirop_vision/data/TOD/milk_step10","milk","ply");
    initParam();
}

TODDetector::~TODDetector()
{
    delete TCCVSION;
    delete LineDeTool;
}

int TODDetector::loadData(const std::string path, const std::string objectName){
//    TCCVSION->LoadModel(path);
//    cout <<"path： " << path<< endl;
//    cout <<"objectName：" <<objectName<<endl;
//LineDeTool->loadData(path , objectName);
    return 0;
}
void TODDetector::setColorImg(const cv::Mat &inputImg){
    if( inputImg.channels() != 3){
        std::cout << "************************** "<<std::endl;
        std::cout << "warning the channel num is no "<< inputImg.channels()<< " "<< inputImg.depth()<<std::endl;
//        assert(inputImg.channels() == 3);

       std::cout <<"image size   "<<inputImg.cols<<" "<< inputImg.rows<<std::endl;
    }

    //这里需要备注一下 识别的颜色类型为 RGB 通道

    ColorShow = inputImg.clone();
    if( inputImg.rows != ColorHeight || inputImg.cols != ColorWidth){
        cv::resize(ColorShow,ColorImg,Size(ColorWidth, ColorHeight));
    }
    cv::cvtColor(ColorImg, ColorImg, COLOR_BGR2RGB);

}
void TODDetector::setDepthImg(const cv::Mat &inputImg){
    cout<< "setDepthImg "<< inputImg.depth()<<endl;
    DepthImg = inputImg;
    DepthShow = DepthImg.clone();
}

int TODDetector::getResult(std::vector<pose> &poses){
    poses.clear();
    pose p1;
    if(outposedd.size() == 0 ){
        p1.objectName = ThisDetectionName;
        p1.position.x = 0;
        p1.position.y = 0;
        p1.position.z = 0;
        p1.quaternion.x = 0;
        p1.quaternion.y = 0;
        p1.quaternion.z = 0;
        p1.quaternion.w = 0;
        poses.push_back(p1);
        return -1;

    }else{
        // detection ok
        poses = outposedd;
    }


    return 0;
}

int TODDetector::getName(std::string &name){
    name = this->ThisDetectionName;
    return 0;
}

int TODDetector::isMultiDetector(){
    
    return 0;
}

ENTITY_TYPE TODDetector::getEntityType(){
    
    return CPP;
}

int TODDetector::initParam(){
    ThisDetectionName = "TOD";
    ColorWidth = 480;
    ColorHeight = 270;
    if( TCCVSION != nullptr)
        TCCVSION->init(ColorWidth, ColorHeight);

    TCCVSION->LoadModel("/home/fshs/KongWork/pb/frozen_inference_graph-20190713-2.pb");
    name = "TODDetection";
    outPose = vector<float>(6);
    return 0;
}

/**
 * 点云分割的区域 比例变换
 */
void TODDetector::ScalarPoint(const Point& p01, const Point& p02, Point& p11, Point& p12){
    float  scalarX = static_cast<float>(DepthImg.cols/this->ColorWidth);
    float  scalarY = static_cast<float>(DepthImg.rows/this->ColorHeight);
    p11.x = static_cast<int>(scalarX * p01.x);
    p12.x = static_cast<int>(scalarX * p02.x);

    p11.y = static_cast<int>(scalarY * p01.y);
    p12.y = static_cast<int>(scalarY * p02.y);
//    cout << "p11 point :"<<p11.x <<" "<<p11.y<<endl;
//    cout << "p12 point :"<<p12.x <<" "<<p12.y<<endl;

}
int TODDetector::detection(){

    //深度学习的 二维图像的识别
    outPose.clear();


//    resize(ColorImg, ColorImg, Size(960,540));
//    cvtColor(ColorImg,ColorImg ,COLOR_RGB2BGR);
//    imwrite("/home/fshs/KongWork/hirop/build/depthTest.jpg", DepthImg);
//    imwrite("/home/fshs/KongWork/hirop/build/ColorImg.png", ColorImg);

    IDebug("%s, %d, %d", "ColorImg size ", ColorImg.cols, ColorImg.rows);
    int rtn = TCCVSION->detection(ColorImg, outPose);
    outposedd.clear();
    outPose.clear();

    if( rtn  < 0 ){
        outPose.clear();
        cout <<  "nothing detection"<<endl;
        return -1;
    }
    cout << "Find the target...... "<<endl;

//    //点云分割 深度图的分割
    int x1 =outPose[2];int y1 =outPose[3];
    int x2 =outPose[4];int y2 =outPose[5];

    Point p11(x1,y1), p12(x2, y2), p21, p22;
    ScalarPoint(p11,p12, p21, p22);
    depthRect = Rect(p21,p22);
    Rect rRoi = Rect(depthRect.x, depthRect.y, depthRect.width*1.2 ,depthRect.height);
    depthRect = rRoi;

    cv::Mat ColorTemp = ColorShow(depthRect);
    cv::Mat DepthTemp = DepthShow(depthRect);
    cv::Mat DepthZero = cv::Mat::zeros(Size(ColorShow.cols, ColorShow.rows), CV_16UC1);
    cv::Mat ColorZero = cv::Mat::zeros(Size(ColorShow.cols, ColorShow.rows), CV_8UC3);
    cv::Mat DepthZeroR = DepthZero(depthRect);
    cv::Mat ColorZeroR = ColorZero(depthRect);

    DepthTemp.copyTo(DepthZeroR);
    ColorTemp.copyTo(ColorZeroR);

    index ++;
    FileStorage fs("/home/fshs/KongWork/data-test/depth/depthTest" +toString(index)+ ".xml", FileStorage::WRITE); //填入写操作
    fs<< "depth"<<DepthZero;
    fs.release();
    imwrite("/home/fshs/KongWork/data-test/other/ColorTemp" +toString(index)+ ".jpg", ColorTemp);
    imwrite("/home/fshs/KongWork/data-test/other/DepthZero" +toString(index)+ ".png", DepthZero);
    imwrite("/home/fshs/KongWork/data-test/JPEGImages/ColorZero" +toString(index)+ ".png", ColorZero);
//    cv::Mat RoiDepth = DepthImg(depthRect);
    //linemod 的三维图像识别
     LineDeTool->setColorImg(ColorZero);
     LineDeTool->setDepthImg(DepthZero);
     if( 0 == LineDeTool->detection()){

        LineDeTool->getResult(outposedd);
        IDebug("%s ", " find ................");
     }else{
        IDebug("%s ", " non - find ................");
     }
    // LineDeTool->getResult()
    return 0;
}

H_EXPORT_PLUGIN(TODDetector, "TODDetector", "1.0")

