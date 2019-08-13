#include <nav/motion.h>

using namespace hirop::navigation;

Motion::~Motion(){

}

int Motion::setOnFinishedListener(OnFinishedListener *lister){

    if(lister == NULL)
        return -1;

    _listers.push_back(lister);
    return 0;
}
