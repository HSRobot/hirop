#include "vision/trainer.h"
#include <iostream>

using namespace hirop_vision;

Trainer::Trainer(){
    trainThr = NULL;
    loader = new CppLoader();
    listener = NULL;
}

int Trainer::setTrainConfig(std::string fileName){

    std::string trainerName;
    std::string objectName;

    // 获取配置信息
    this->config = new Configure(fileName);

    // 获取训练物体名称
    if(config->getObjectName(objectName))
        goto _failed;

    // 获取训练器名称
    if(config->getTrainerName(trainerName))
        goto _failed;

    // 加载训练器实例
    this->trainer = loader->loadTrainer(trainerName);

    if(this->trainer == NULL){
        std::cerr << "loading trainer: SimapleTrainer  error" << std::endl;
    }

    return 0;

    // 获取配置时出现的错误
_failed:
    std::cerr << "Paser config file error" << std::endl;
    return -1;
}

int Trainer::train(){

    // 训练线程已经存在
    if(trainThr != NULL){
        std::cerr << "can't not start training: train thread was running" << std::endl;
        return -1;
    }

    // 相关训练器未加载
    if(trainer == NULL){
        std::cerr << "can't not start training: trainer was not loaded" << std::endl;
        return -1;
    }

    // 创建线程
    boost::function0<int> f =  boost::bind(&Trainer::__train,this);
    trainThr = new boost::thread(f);

    // 启动线程
    trainThr->join();

    return 0;
}


int Trainer::setOnStateChangeListener(TrainStateListener *listener){
    if(listener == NULL){
        std::cerr << "set state listener was error: listener can't be NULL" << std::endl;
        return -1;
    }
    this->listener = listener;
    return 0;
}

int Trainer::__train(){
    int ret;
    std::string trainName;
    std::string path;

    // 进行训练
    ret = trainer->train();
    trainer->getName(trainName);

    // 调用训练完成回调函数
    if(listener != NULL)
        listener->onTrainDone(trainName, ret);

    // 训练成功 保存结果
    if(ret == 0){
        __genPath(path);
        trainer->saveData(path);
    }

    // 删除线程对象
    delete trainThr;
    return 0;
}

int Trainer::__genPath(std::string &path){

    // 数据保存的前缀路径
    std::string prefix = "/home/fshs/hirop_vision/data/";
    std::string objectName;
    std::string trainerName;

    config->getObjectName(objectName);
    config->getTrainerName(trainerName);

    // TODO: 检查路径是否存在，如果无则生成。检查路径是否有读写权限，如无则报错。
    path = prefix + trainerName + "/" + objectName + "/";

    return 0;

}
