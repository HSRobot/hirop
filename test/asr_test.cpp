#include <asr/hirop_asr.h>

using namespace HIROP::ASR;

int main(){

    HSpeechRecognition* asr = HSpeechRecognition::getInstance();

    asr->init();

    asr->startListen();


    while(1);
}
