#include <TODDetector.h>
#define DEBUG
#include <utils/idebug.h>
#include <string>
//#include <opencv2/core/eigen/hpp>
template <typename T>
string toString(T& t){
    ostringstream os;
    os << t;
    return os.str();
}

TODDetector::TODDetector():CBaseDetector("TODDetector", false){
    LineDeTool = new LinemodDetector();
    TCCVSION = new TensorflowCCVisoin();
    index = 0;

    initParam();
}

TODDetector::~TODDetector()
{
    delete TCCVSION;
    delete LineDeTool;
}

int TODDetector::loadData(const std::string path, const std::string objectName){
    cout <<"path： " << path<< endl;
    cout <<"objectName：" <<objectName<<endl;
    IDebug("ColorWidth: %d ColorHeight: %d ",ColorWidth, ColorHeight);


    if( TCCVSION != nullptr)
        TCCVSION->init(ColorWidth, ColorHeight);
    else{
        cerr << "tensorflow init error !"<<endl;
        return -1;
    }

    TCCVSION->LoadModel(this->pbPath);
    LineDeTool->loadData(path, objectName, "stl");
    return 0;
}

int TODDetector::parseConfig(const YAML::Node &node){
    int rtn = this->readYamlData(node);
    rtn += LineDeTool->readYamlData(node);
    return rtn;
}

void TODDetector::setColorImg(const cv::Mat &inputImg){
    if( inputImg.channels() != 3){
        IErrorPrint("inputImg.channels is no three , it can cause anothor errors %d" ,inputImg.channels());
    }

    IErrorPrint("input image size  %d  %d  %d", inputImg.cols, inputImg.rows,inputImg.depth());
    //这里需要备注一下 识别的颜色类型为 RGB 通道 输入的图像预处理
    ColorShow = inputImg.clone();
    if( inputImg.rows != ColorHeight || inputImg.cols != ColorWidth){
        cv::resize(ColorShow, ColorImg,Size(ColorWidth, ColorHeight));
    }else{
		ColorImg = ColorShow.clone();
	}

    cv::cvtColor(ColorImg, ColorImg, COLOR_BGR2RGB);

}
void TODDetector::setDepthImg(const cv::Mat &inputImg){
    IErrorPrint("setDepthImg : %d ", inputImg.depth());
    DepthImg = inputImg;

    // 深度滤波
//    for( int i = 0; i < DepthImg.rows; i++){
//        for( int j = 0; j < DepthImg.cols; j++){
//            short val = DepthImg.at<ushort>(i,j);
//            if( val > 1000){
//                DepthImg.at<ushort>(i,j) = 0;
//            }
//        }
//    }

    DepthShow = DepthImg.clone();
}

int TODDetector::getResult(std::vector<pose> &poses){
    poses.clear();
    pose p1;
    if(outposedd.size() == 0 ){
        p1.objectName = ThisDetectionName;
        p1.position.x = 0;   p1.position.y = 0;   p1.position.z = 0;
        p1.quaternion.x = 0; p1.quaternion.y = 0;
        p1.quaternion.z = 0; p1.quaternion.w = 0;
        poses.push_back(p1);
        return -1;
    }else{
        // detection ok
        poses = outposedd;
        for(int i =0; i < poses.size() ;i++){
            std::cout << " out: "<< poses[i].position.x<<" "  \
                        << poses[i].position.y<<" " << poses[i].position.z<<" "  \
                        << poses[i].quaternion.x<<" " <<poses[i].quaternion.y<<"  " \
                        <<poses[i].quaternion.z<<"  "  <<poses[i].quaternion.w <<std::endl;
        }
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

/**
 * @brief TODDetector::initParam 数据初始化
 * @return
 */
int TODDetector::initParam(){
    ThisDetectionName = "TOD";

    TODName = {"ColorWidth", "ColorHeight","camera_factor" ,"pbModel", "3DModel"};
    name = "TODDetection";
    outPose = vector<float>(6);
    return 0;
}

/**
 * 点云分割的区域 比例变换
 */
void TODDetector::ScalarPoint(const Point& p01, const Point& p02, Point& p11, Point& p12){
    float  scalarX = static_cast<float>(DepthShow.cols/this->ColorWidth);
    float  scalarY = static_cast<float>(DepthShow.rows/this->ColorHeight);
    p11.x = static_cast<int>(scalarX * p01.x);
    p12.x = static_cast<int>(scalarX * p02.x);
    p11.y = static_cast<int>(scalarY * p01.y);
    p12.y = static_cast<int>(scalarY * p02.y);

}

int TODDetector::DepthFitter(Rect rRoi, const Mat& DepthImg, double& MeanDepth)
{
    vector<double> disArray;
    int centerX = rRoi.x + rRoi.width/2;
    int centerY = rRoi.y + rRoi.height/2;
    cout << centerX<< " "<<centerY <<endl;
    for(int i = -1; i < 2 ; i++){
        for(int j = -1; j < 2; j++)
        {
            double  d =(double)DepthImg.at<short>(centerY + i, centerX + j )/1000;
            cout << "d "<<d<<endl;
            if(d <= 0 ) continue;
            disArray.push_back(d);
        }
    }
    double SumDistance = 0;
    for( int i = 0; i < disArray.size(); i++){
        SumDistance += disArray[i];
    }
    cout << "sum "<< SumDistance <<endl;
    MeanDepth = SumDistance/disArray.size();

    return 0;
}

int TODDetector::readYamlData(const YAML::Node &node)
{
    try{
        this->ColorWidth = node["TOD"]["ColorWidth"].as<int>();
        this->ColorHeight = node["TOD"]["ColorHeight"].as<int>();
        this->pbPath = node["TOD"]["pbPath"].as<std::string>();
    }catch(YAML::Exception e)
    {
        std::cerr<<"TOD YAML:" << e.msg<<std::endl;
        return -1;
    }
    std::cout <<"pbPath "<<pbPath<<std::endl;
    return 0;


}
int TODDetector::detection(){

    //深度学习的 二维图像的识别
    outPose.clear();

    IDebug("%s, %d, %d", "TF detection ColorImg size ", ColorImg.cols, ColorImg.rows);
    int rtn = TCCVSION->detection(ColorImg, outPose);
    outposedd.clear();
    outPose.clear();

    if( rtn  < 0 ){
        outPose.clear();
        IErrorPrint("%s","nothing detection" );
        return -1;
    }
    cout << "Find the target...... "<<endl;

    //点云分割 深度图的分割
    int x1 =outPose[2];int y1 =outPose[3];
    int x2 =outPose[4];int y2 =outPose[5];

    Point p11(x1,y1), p12(x2, y2), p21, p22;
    ScalarPoint(p11,p12, p21, p22);
    depthRect = Rect(p21,p22);
    Rect rRoi = Rect(depthRect.x, depthRect.y, depthRect.width ,depthRect.height);
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
    IDebug("%s, %d, %d", "LineDeTool ColorImg size ", ColorZero.cols, ColorZero.rows);


	////std::cout << externM* cameraIntriM*input<<std::endl;
    //linemod 的三维图像识别
     LineDeTool->setColorImg(ColorZero);
     LineDeTool->setDepthImg(DepthZero);
     if( 0 == LineDeTool->detection()){

        LineDeTool->getResult(outposedd);

        for( int i = 0; i < this->outposedd.size(); i++){
            Eigen::Quaterniond CubeRotationShow =  Eigen::Quaterniond(outposedd[i].quaternion.w, outposedd[i].quaternion.x,\
                                                                  outposedd[i].quaternion.y,outposedd[i].quaternion.z);
            //"输出 Z-Y-X，即RPY  "
            Eigen::Vector3d eulerAngle= CubeRotationShow.matrix().eulerAngles(2,1,0);
            CubeRotationShow = Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ()) *  \
                    Eigen::AngleAxisd(eulerAngle[1]+1.57, Eigen::Vector3d::UnitY()) *  \
                    Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX());

            outposedd[i].quaternion.x = CubeRotationShow.x();
            outposedd[i].quaternion.y = CubeRotationShow.y();
            outposedd[i].quaternion.z = CubeRotationShow.z();
            outposedd[i].quaternion.w = CubeRotationShow.w();
        }

        IDebug("%s ", " find ................");
     }else{
//		outposedd.clear();
        IDebug("%s ", " linemod non-find  .............");
		return -1;
     }



    double MeanDepth;
    DepthFitter(rRoi, DepthImg, MeanDepth);
    outposedd[0].position.z = MeanDepth;


    return 0;
}

H_EXPORT_PLUGIN(TODDetector, "TODDetector", "1.0")

