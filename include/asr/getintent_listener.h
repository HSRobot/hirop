#pragma once

#include <iostream>

namespace HIROP{

namespace ASR{

class GetIntentListener{
public:

    virtual void onGetIntent(const char* intent) = 0;

};

}

}
