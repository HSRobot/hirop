#pragma once

#include "aiui_asr.h"
#include "audio_source.h"

#include "getintent_listener.h"

namespace HIROP{

namespace ASR {

class HSpeechRecognition : public AudioSourceListener{

public:
    static HSpeechRecognition *getInstance();

    /**
     * @brief startListen   开始监听用户的意图
     * @return              0 监听成功 -1 失败
     */
    int startListen();

    /**
     * @brief stopListen    停止监听用户的意图
     * @return              0 停止成功 -1 失败
     */
    int stopListen();

    /**
     * @brief setIntentListener 设置意图监听者
     * @return                  0 设置成功 -1 设置失败
     */
    void setIntentListener(GetIntentListener *listener);

    /**
     * @brief updateAudioData   更新音频数据
     * @param buf               音频数据
     * @param size              音频数据大小
     */
    void updateAudioData(char *buf, int size);

    /**
     * @brief init      初始化
     * @return
     */
    int init();

    /**
     * @brief isListening   获取当前是否在监听
     * @return              true if listening
     */
    bool isListening() {return _islistening;}

private:
    /**
     * @brief __listening   监听中的线程
     */
    void __listening();

private:
    /**
     * @brief 构造函数
     */
    HSpeechRecognition();

    /**
     * @brief 析构函数
     */
    ~HSpeechRecognition();

    /**
     * @brief _insatnce 唯一实例
     */
    static HSpeechRecognition* _instance;

    /**
     * @brief aiui  aiui的实例
     */
    AIUIAsr *aiui;

    /**
     * @brief _islistening   是否开启了监听
     */
    bool _islistening;

};

}

}

