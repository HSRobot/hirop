#include "vision/trainer.h"
#include "vision/vision.h"

#include <iostream>
using namespace std;
using namespace hirop_vision;
int main(){
    int i;
    Trainer t;
    t.setTrainConfig("test.yaml");
    t.train();
    std::cin >> i;
}
