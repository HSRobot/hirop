#pragma once

namespace HIROP{

namespace ASR{

class AudioSourceListener{

public:
    virtual void updateAudioData(char *buf, int size) = 0;

};

}

}
