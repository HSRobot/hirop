#include <asr/aiui_asr.h>
#include "jsoncpp/json/json.h"
#include <string.h>

#include "aiui_config.h"

using namespace VA;
using namespace HIROP::ASR;

AIUIAsr::AIUIAsr(){


}

int AIUIAsr::init(){

    AIUISetting::setAIUIDir(AIUI_DIR);
    AIUISetting::initLogger(AIUI_LOG);

    sleep(1);

    if(createAgent())
        return -1;

    wakeup();
    return 0;
}

int AIUIAsr::createAgent(){

    string appid = AIUI_APPID;
    Json::Value paramJson;
    Json::Value appidJson;

    appidJson["appid"] = appid;

    string fileParam = FileUtil::readFileAsString(AIUI_CONFIG_FILE);
    Json::Reader reader;

    if(reader.parse(fileParam, paramJson, false))
    {
        paramJson["login"] = appidJson;

        //for ivw support
        string wakeup_mode = paramJson["speech"]["wakeup_mode"].asString();

        //如果在aiui.cfg中设置了唤醒模式为ivw唤醒，那么需要对设置的唤醒资源路径作处理，并且设置唤醒的libmsc.so的路径为当前路径
        if(wakeup_mode == "ivw")
        {
            //readme中有说明，使用libmsc.so唤醒库，需要调用MSPLogin()先登录
            //string lgiparams = "appid=5a950e5a,engine_start=ivw";
            //MSPLogin(NULL, NULL, lgiparams.c_str());
            string ivw_res_path = paramJson["ivw"]["res_path"].asString();
            if(!ivw_res_path.empty())
            {
                ivw_res_path = "fo|" + ivw_res_path;
                paramJson["ivw"]["res_path"] = ivw_res_path;
            }

            string ivw_lib_path = "libmsc.so";

            paramJson["ivw"]["msc_lib_path"] = ivw_lib_path;
        }
        //end

        Json::FastWriter writer;
        string paramStr = writer.write(paramJson);
        agent = IAIUIAgent::createAgent(paramStr.c_str(), &listener);
        return 0;
    }
    else
    {
        cout << "aiui.cfg has something wrong!" << endl;
        return -1;
    }
}

void AIUIAsr::wakeup()
{
    if (NULL != agent)
    {
        IAIUIMessage * wakeupMsg = IAIUIMessage::create(AIUIConstant::CMD_WAKEUP);
        agent->sendMessage(wakeupMsg);
        wakeupMsg->destroy();
    }
}

void AIUIAsr::writeText(){

    if (NULL != agent)
    {
        string text = "刘德华的歌。";
        // textData内存会在Message在内部处理完后自动release掉
        Buffer* textData = Buffer::alloc(text.length());
        text.copy((char*) textData->data(), text.length());

        IAIUIMessage * writeMsg = IAIUIMessage::create(AIUIConstant::CMD_WRITE,
                                                       0,0, "data_type=text", textData);

        agent->sendMessage(writeMsg);
        writeMsg->destroy();
    }
}

int AIUIAsr::writeAudioData(void *data, int size){

    Buffer* buffer = Buffer::alloc(size);//申请的内存会在sdk内部释放
    memcpy(buffer->data(), data, size);

    /**
     * @brief 这里需要注意音频格式的问题
     */
    IAIUIMessage * writeMsg = IAIUIMessage::create(AIUIConstant::CMD_WRITE,
                                                   0, 0,  "data_type=audio,sample_rate=16000", buffer);

    /**
     * 将音频数据发送给AIUI
     */
    if (NULL != agent)
    {
        agent->sendMessage(writeMsg);
        return -1;
    }

    writeMsg->destroy();

    /**
     *  这里没有发送停止的指令，不知道是否有问题。
     */

    return 0;
}

int AIUIAsr::stopWriteAudioData(){

    IAIUIMessage * stopWrite = IAIUIMessage::create(AIUIConstant::CMD_STOP_WRITE,
                                                    0, 0, "data_type=audio,sample_rate=16000");

    if (NULL != agent)
    {
        agent->sendMessage(stopWrite);
    }
    stopWrite->destroy();

}

AiuiListener::AiuiListener()
{
    std::cout << "in AiuiListener" << std::endl;
    mTtsFileHelper = new FileUtil::DataFileHelper("");
    _listener = NULL;
}

AiuiListener::~AiuiListener()
{
    std::cout << "in ~AiuiListener" << std::endl;

    if (mTtsFileHelper != NULL)
    {
        delete mTtsFileHelper;
        mTtsFileHelper = NULL;
    }
}

string mSyncSid;

//事件回调接口，SDK状态，文本，语义结果等都是通过该接口抛出
void AiuiListener::onEvent(const IAIUIEvent& event) const
{
    switch (event.getEventType()) {
    //SDK 状态回调
    case AIUIConstant::EVENT_STATE:
    {
        switch (event.getArg1()) {
        case AIUIConstant::STATE_IDLE:
        {
            cout << "EVENT_STATE:" << "IDLE" << endl;
        } break;

        case AIUIConstant::STATE_READY:
        {
            cout << "EVENT_STATE:" << "READY" << endl;
        } break;

        case AIUIConstant::STATE_WORKING:
        {
            cout << "EVENT_STATE:" << "WORKING" << endl;
        } break;
        }
    } break;

        //唤醒事件回调
    case AIUIConstant::EVENT_WAKEUP:
    {
        cout << "EVENT_WAKEUP:" << event.getInfo() << endl;
    } break;

        //休眠事件回调
    case AIUIConstant::EVENT_SLEEP:
    {
        cout << "EVENT_SLEEP:arg1=" << event.getArg1() << endl;
    } break;

        //VAD事件回调，如找到前后端点
    case AIUIConstant::EVENT_VAD:
    {
        switch (event.getArg1()) {
        case AIUIConstant::VAD_BOS:
        {
            cout << "EVENT_VAD:" << "BOS" << endl;
        } break;

        case AIUIConstant::VAD_EOS:
        {
            cout << "EVENT_VAD:" << "EOS" << endl;
        } break;

        case AIUIConstant::VAD_VOL:
        {

        } break;
        }
    } break;

        //最重要的结果事件回调
    case AIUIConstant::EVENT_RESULT:
    {
        Json::Value bizParamJson;
        Json::Reader reader;

        Json::Value intentRoot;

        if (!reader.parse(event.getInfo(), bizParamJson, false)) {
            cout << "parse error!" << endl << event.getInfo() << endl;
            break;
        }
        Json::Value data = (bizParamJson["data"])[0];
        Json::Value params = data["params"];
        Json::Value content = (data["content"])[0];
        string sub = params["sub"].asString();

        if (sub == "nlp")
        {
            Json::Value empty;
            Json::Value contentId = content.get("cnt_id", empty);

            if (contentId.empty())
            {
                cout << "Content Id is empty" << endl;
                break;
            }

            string cnt_id = contentId.asString();
            int dataLen = 0;
            const char* buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
            string resultStr;

            if (NULL != buffer)
            {
                resultStr = string(buffer, dataLen);
                cout << resultStr << endl;

                if (!reader.parse(resultStr, intentRoot, false)) {
                    cout << "intent root parse error!" << endl;
                    break;
                }

                Json::Value semanticRoot = (intentRoot["intent"]["semantic"])[0];
                string semString;
                if(!semanticRoot.empty() && _listener != NULL){
                    semString = semanticRoot.toStyledString();
                    _listener->onGetIntent(semString.c_str());
                }

            }
        }

    }
        break;

        //上传资源数据的返回结果
    case AIUIConstant::EVENT_CMD_RETURN:
    {
        //cout << "onEvent --> EVENT_CMD_RETURN: arg1 is " << event.getArg1() << endl;
        if (AIUIConstant::CMD_SYNC == event.getArg1())
        {
            int retcode = event.getArg2();
            int dtype = event.getData()->getInt("sync_dtype", -1);

            //cout << "onEvent --> EVENT_CMD_RETURN: dtype is " << dtype << endl;

            switch (dtype)
            {
            case AIUIConstant::SYNC_DATA_STATUS:
                break;

            case AIUIConstant::SYNC_DATA_ACCOUNT:
                break;

            case AIUIConstant::SYNC_DATA_SCHEMA:
            {
                string sid = event.getData()->getString("sid", "");
                string tag = event.getData()->getString("tag", "");

                mSyncSid = sid;

                if (AIUIConstant::SUCCESS == retcode)
                {
                    cout << "sync schema success." << endl;
                } else {
                    cout << "sync schema error=" << retcode << endl;
                }

                cout << "sid=" << sid << endl;
                cout << "tag=" << tag << endl;
            } break;

            case AIUIConstant::SYNC_DATA_SPEAKABLE:
                break;

            case AIUIConstant::SYNC_DATA_QUERY://查询结果
            {
                if (AIUIConstant::SUCCESS == retcode)
                {
                    cout << "sync status success" << endl;
                } else {
                    cout << "sync status error=" << retcode << endl;
                }
            } break;
            }
        } else if (AIUIConstant::CMD_QUERY_SYNC_STATUS == event.getArg1()) {
            int syncType = event.getData()->getInt("sync_dtype", -1);
            if (AIUIConstant::SYNC_DATA_QUERY == syncType)
            {
                string result = event.getData()->getString("result", "");
                cout << "result:" << result << endl;

                if (0 == event.getArg2())
                {
                    cout << "sync status:success." << endl;
                } else {
                    cout << "sync status error:" << event.getArg2() << endl;
                }
            }
        } else if (AIUIConstant::CMD_BUILD_GRAMMAR == event.getArg1()) {
            if (event.getArg2() == 0)
            {
                cout << "build grammar success." << endl;
            }
            else
            {
                cout << "build grammar error, errcode = " << event.getArg2() << endl;
                cout << "error reasion is " << event.getInfo() << endl;
            }
        } else if (AIUIConstant::CMD_UPDATE_LOCAL_LEXICON == event.getArg1()) {
            if (event.getArg2() == 0)
            {
                cout << "update lexicon success" << endl;
            }
            else
            {
                cout << "update lexicon error, errcode = " << event.getArg2() << endl;
                cout << "error reasion is " << event.getInfo() << endl;
            }
        }
    } break;

    case AIUIConstant::EVENT_ERROR:
    {
        cout << "EVENT_ERROR:" << event.getArg1() << endl;
        cout << " ERROR info is " << event.getInfo() << endl;
    } break;
    }
}
