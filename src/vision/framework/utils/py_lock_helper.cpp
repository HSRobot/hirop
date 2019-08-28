#include "utils/py_lock_helper.h"

PyLockHelper::PyLockHelper(){

    nStatus = 0;
#ifdef WITH_PYTHON3
    nStatus = PyGILState_Check();
#endif
    if( !nStatus)
        gstate = PyGILState_Ensure();
}

PyLockHelper::~PyLockHelper(){

    if( gstate )
        PyGILState_Release(gstate);
}
