#include <asr/hirop_asr.h>
#include <string.h>

using namespace HIROP::ASR;

HSpeechRecognition* HSpeechRecognition::_instance = NULL;

HSpeechRecognition* HSpeechRecognition::getInstance(){

    if(!_instance){
        _instance = new HSpeechRecognition();
    }

    return _instance;
}

HSpeechRecognition::HSpeechRecognition(){

    aiui = new AIUIAsr();

}

int HSpeechRecognition::init(){
    aiui->init();
}

int HSpeechRecognition::isError()
{
    return aiui->isError();
}

int HSpeechRecognition::startListen(){

    aiui->resetError();
    _islistening = true;

}

int HSpeechRecognition::stopListen(){

    _islistening = false;
    aiui->stopWriteAudioData();

}

void HSpeechRecognition::setIntentListener(GetIntentListener *listener){
    aiui->setIntentListener(listener);
}


void HSpeechRecognition::updateAudioData(char *buf, int size){

    if(size > 0 && isListening()){
        aiui->writeAudioData(buf, size);
    }
}



