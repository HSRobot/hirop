#include "vision/trainer.h"
#include "vision/vision.h"
#include "opencv2/core.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>
using namespace std;
using namespace cv;
using namespace hirop_vision;

map<string ,float> TODData;
Mat cameraIntriM, coffeM;
map<string ,float> LinemodData;
Mat tmpMat_;
void readYamlData(){

}

int main(){
    int i;

    map<string ,float> TODMap;
    Mat cameraIntriM, coffeM;
    map<string ,float> LinemodMap;
    Mat tmpMat_;
    const vector<string> TODName= { "ColorWidth", "ColorHeight","camera_factor" ,"pbModel", "3DModel"};
    vector<string> LinemodData=   { "param_n_points_",   "param_angle_step_",     "param_radius_min_",\
                                        "param_radius_max_", "param_radius_step_",    "param_width_" ,\
                                        "param_height_",     "param_focal_length_x_", "param_focal_length_y_,"\
                                        "param_near_",       "param_near_","param_far_"};
    for( int i = 0; i < TODName.size(); i++){
       TODMap.insert(map<string, float>::value_type(TODName[i], 1));
    }

    for( int i = 0; i < LinemodData.size(); i++){
       LinemodMap.insert(map<string, float>::value_type(LinemodData[i], 1));
    }
    cout<< "TODData.size :  "<<TODMap.size()<<endl;
    cout<< "LinemodMap.size :  "<<LinemodMap.size()<<endl;

    for(int i = 0 ;i < TODMap.size(); i++ ){
        cout << TODName[i] << " "<<TODMap.at(TODName[i])<<endl;
    }
//    system("pause");
    return 0;

}
