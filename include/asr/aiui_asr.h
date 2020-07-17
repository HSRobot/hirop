#pragma once

#include <iostream>

#include <aiui/AIUI.h>

#include "FileUtil.h"

#include "getintent_listener.h"

using namespace aiui;
using namespace std;

#define CFG_FILE_PATH "./AIUI/cfg/aiui.cfg"

namespace HIROP{

namespace ASR{


class AiuiListener : public IAIUIListener
{
private:
    FileUtil::DataFileHelper* mTtsFileHelper;

    GetIntentListener *_listener;
public:
    void onEvent(const IAIUIEvent& event) const;

    void setIntentListener(GetIntentListener *listener) {_listener = listener;}

    int getListenState(){return _listener->getAiuiErrorStatus();}

    void resetListenState(){_listener->setAiuiErrorStatus(0);}

    AiuiListener();


    ~AiuiListener();
};


/**
 * @brief 科大讯飞AIUI的封装类
 */
class AIUIAsr{

public:
    /**
     * @brief AIUIAsr   构造函数
     */
    AIUIAsr();

    /**
     * @brief init      初始化科大讯飞的AIUI，包括创建代理和进行唤醒
     * @return          0 初始化成功 -1初始化失败
     */
    int init();

    /**
     * @brief writeText 输入文字进行测试
     */
    void writeText();

    /**
     * @brief updateAudioData    写入音频数据
     * @return                  0 成功 -1 失败
     */
    int writeAudioData(void *data, int size);

    /**
     * @brief stopWriteAudioData    停止写入数据
     * @return      0 成功 -1 失败
     */
    int stopWriteAudioData();

    /**
     * @brief setIntentListener     设置意图监听者
     * @param listener              监听者对象
     */
    void setIntentListener(GetIntentListener *getIntentListener) {_getIntentListener = getIntentListener; listener.setIntentListener(getIntentListener);}


    /**
     * @brief isError
     * @return
     */
    int isError();

    /**
     * @brief resetError
     */
    void resetError();

private:
    /**
     * @brief createAgent   创建AIUI的代理
     * @return              0 成功 -1 失败
     */
    int createAgent();

    /**
     * @brief wakeup        唤醒科大讯飞的AIUI
     */
    void wakeup();

    /**
     * @brief agent         代理类的实例
     */
    IAIUIAgent* agent;

    /**
     * @brief listener      AIUI事件的监听者
     */
    AiuiListener listener;

    /**
     * @brief getIntentListener 意图监听器
     */
    GetIntentListener *_getIntentListener;

};

}

}
