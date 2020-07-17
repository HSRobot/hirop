#pragma once

#include <iostream>

namespace HIROP{

namespace ASR{

class GetIntentListener{
public:
    GetIntentListener():errorState(0){}
public:
    virtual void onGetIntent(const char* intent) = 0;
    void setAiuiErrorStatus(int errorState){this->errorState = errorState; }
    int getAiuiErrorStatus(){

        return errorState;
    }
private:
    int errorState;

};

}

}
