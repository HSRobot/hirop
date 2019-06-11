#ifndef __LINEMOD_TRAINER_H__
#define __LINEMOD_TRAINER_H__

#include "itrainer.h"
#include "vision.h"

using namespace hirop_vision;

class LinemodTrainer:public ITrainer{

public:
    LinemodTrainer();
    int train();
    int feedback();
    int parseConfig();
    int saveData(std::string path);
    int deleteData();
    int getName(std::string &name);


};

#endif // LINEMOD_TRAINER_H
