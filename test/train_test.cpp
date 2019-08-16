#include "vision/trainer.h"
#include "vision/vision.h"
#include "opencv2/core.hpp"
#include <iostream>
using namespace std;
using namespace cv;
using namespace hirop_vision;
int main(){
    int i;
    Trainer t;
    t.setTrainConfig("/home/fshs/work/hirop/test/test3.yaml");
    t.train();
    std::cin >> i;
}
