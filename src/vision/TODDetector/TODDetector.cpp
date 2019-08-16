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
//    TCCVSION->LoadModel(path);
    cout <<"path： " << path<< endl;
    cout <<"objectName：" <<objectName<<endl;
	if( TCCVSION == nullptr ){
		TCCVSION = new TensorflowCCVisoin();
	}

	if(LineDeTool == nullptr ){
		LineDeTool = new LinemodDetector();
	}	
//LineDeTool->loadDatfaster_rcnn_inception_resnet_v2_atrous_coco-15000-20190726a(path , objectName);
    return 0;
}
void TODDetector::setColorImg(const cv::Mat &inputImg){
	std::cout << inputImg.channels()<<std::endl;
    if( inputImg.channels() != 3){
        std::cout << "************************** "<<std::endl;
//        assert(inputImg.channels() == 3);
    }

    std::cout <<"image size   "<<inputImg.cols<<" "<< inputImg.rows<<std::endl;

    //这里需要备注一下 识别的颜色类型为 RGB 通道

    ColorShow = inputImg.clone();
    if( inputImg.rows != ColorHeight || inputImg.cols != ColorWidth){
        cv::resize(ColorShow,ColorImg,Size(ColorWidth, ColorHeight));
    }else{
		ColorImg = ColorShow.clone();
	}
    cv::cvtColor(ColorImg, ColorImg, COLOR_BGR2RGB);

}
void TODDetector::setDepthImg(const cv::Mat &inputImg){
    cout<< "setDepthImg "<< inputImg.depth()<<endl;
    DepthImg = inputImg;

    for( int i = 0; i < DepthImg.rows; i++){
        for( int j = 0; j < DepthImg.cols; j++){
            short val = DepthImg.at<ushort>(i,j);
            if( val > 1000){
                DepthImg.at<ushort>(i,j) = 0;
            }
        }
    }

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
        for(int i =0; i < poses.size() ;i++){
            std::cout << " out: "<< poses[i].position.x<<" "  \
                        << poses[i].position.y<<" "  \
                        << poses[i].position.z<<" "  \
                        << poses[i].quaternion.x<<" "  \
                        <<poses[i].quaternion.y<<"  " \
                        <<poses[i].quaternion.z<<"  " \
                        <<poses[i].quaternion.w <<std::endl;
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

int TODDetector::initParam(){
    ThisDetectionName = "TOD";
    ColorWidth = 640;
    ColorHeight = 480;
	if( TCCVSION != nullptr)
        TCCVSION->init(ColorWidth, ColorHeight);


    LineDeTool->loadData("/home/fshs/hirop_vision/data/TOD/coke","milk","stl");
    TCCVSION->LoadModel("/home/fshs/KongWork/pb/20190814-coke-1.pb");
//    TCCVSION->LoadModel("/home/fshs/KongWork/pb/faster_rcnn_inception_resnet_v2_atrous_coco-15000-20190726.pb");
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
//    cout << "p11 point :"<<p11.x <<" "<<p11.y<<endl;
//    cout << "p12 point :"<<p12.x <<" "<<p12.y<<endl;

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

//    //点云分割 深度图的分割
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

	//pose p1;
	//p1.position.x = (float)(x1 + x2)/2;
	//p1.position.y = (float)(y1 + y2)/2;
	//p1.position.z = (float)DepthZero.at<ushort>(DepthShow.rows/2, DepthShow.cols/2)/1000;
	//outposedd.push_back(p1);
//	cv::Mat cameraIntriM = (cv::Mat_<float>(3,3)<<
//					1006.2,0,614.396, \
//					0,974.716,408.633,\
//					0,0,1);
	cv::Mat cameraIntriM = (cv::Mat_<float>(3,3)<<
					617.94,0,325.894, \
					0,614.1937,247.2783);
	std::cout << cameraIntriM <<std::endl;
	cv::Mat coffeM = (cv::Mat_<float>(5,1)<<
					0.1727,-0.4572,0,0,0);
	//cv::Mat input = (cv::Mat_<float>(4,4)<<
	//				(float)(x1 + x2)/2,0,0,0,
	//			   	(float)(y1 + y2)/2,0,0,0,
	//				1,0,0,0,
	//			   	0,0,0,0);

	float camera_fx = cameraIntriM.at<float>(0,0);
	float camera_fy = cameraIntriM.at<float>(1,1);
	float camera_cx = cameraIntriM.at<float>(0,2);
	float camera_cy = cameraIntriM.at<float>(1,2);
	float camera_factor = 1000;
	std::cout << camera_fx <<" "<<camera_fy<<std::endl;

	std::cout << camera_cx <<" "<<camera_cy<<std::endl;
	//将控制点在世界坐标系的坐标压入容器
	vector<Point2f> ImageP;
	Mat objM;
	ImageP.clear();
	int Xdias = 10, ydias = 10;
	//ImageP.push_back(Point2f(rRoi.x-Xdias ,rRoi.y-ydias));
	//ImageP.push_back(Point2f(rRoi.x-Xdias, rRoi.y+ydias));
	//ImageP.push_back(Point2f(rRoi.x+Xdias, rRoi.y+ydias));
	//ImageP.push_back(Point2f(rRoi.x+Xdias, rRoi.y-ydias));

	ImageP.push_back(Point2f(p21.x+Xdias , p21.y+ydias));
	ImageP.push_back(Point2f(p21.x+Xdias, p22.y-ydias));
	ImageP.push_back(Point2f(p22.x-Xdias, p22.y-ydias));
	ImageP.push_back(Point2f(p22.x-Xdias, p21.y+ydias));


	vector<Point3f> objP;
//	for(int i = 0; i < ImageP.size(); i++){
//		Point2f p = ImageP[i];
//		//short distance = DepthShow.at<short>(p.x, p.y);
		double z = 0.3619;
//		//double z = double(distance)/1000 ;
//		std::cout << p.x << " "<<p.y<<" "<<z<<std::endl;
//		double x = (p.x - camera_cx) * z / camera_fx;
//		double y = (p.y - camera_cy) * z / camera_fy;
//		objP.push_back(Point3f(x,y,z));
//O	}
	
	objP.push_back(Point3f(-0.025 ,0.05, 0));
	objP.push_back(Point3f(-0.025,-0.05,0));
	objP.push_back(Point3f(0.025,-0.05,0));
	objP.push_back(Point3f(0.025,0.05,0));


	std::cout << "*****************"<<std::endl;
	std::cout << Mat(objP)<< std::endl;

	std::cout << "*****************"<<std::endl;
	std::cout << Mat(ImageP)<< std::endl;

	Mat rvec, tvec;
	solvePnP(Mat(objP), Mat(ImageP), cameraIntriM ,coffeM, rvec, tvec);

	std::cout << "*****************"<<std::endl;
	std::cout << rvec<< std::endl;
	std::cout << "*****************"<<std::endl;
	std::cout << tvec<< std::endl;
//	Mat rotM, rotT;
//	Rodrigues(rvec, rotM);  //将旋转向量变换成旋转矩阵
//	Rodrigues(tvec, rotT);

	////std::cout << externM* cameraIntriM*input<<std::endl;
    //linemod 的三维图像识别
     LineDeTool->setColorImg(ColorZero);
     LineDeTool->setDepthImg(DepthZero);
     if( 0 == LineDeTool->detection()){

        LineDeTool->getResult(outposedd);
        IDebug("%s ", " find ................");
     }else{
        IDebug("%s ", " non - find ................");
     }
	pose p1;
	std::cout << tvec.at<double>(1,0)<<std::endl;
	//p1.position.x = tvec.at<float>(0); 
	//p1.position.y = tvec.at<float>(1);
	vector<double> disArray;
	circle(ColorZero, Point(rRoi.x,rRoi.y),50,Scalar(255,255,0) );
	circle(ColorZero, Point(p22.x,p22.y),50,Scalar(255,255,0) );
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
	//p1.position.z = SumDistance / disArray.size();

	cout << " pnp: "<< tvec.at<float>(0)  << " "<< tvec.at<float>(1) <<" "<<p1.position.z<<endl; 
	//outposedd.push_back(p1);

    return 0;
}

H_EXPORT_PLUGIN(TODDetector, "TODDetector", "1.0")

