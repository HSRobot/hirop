#include "utils/py_lock_helper.h"

PyLockHelper::PyLockHelper(){

    nStatus = 0;
   // nStatus = PyGILState_Check();

    if( !nStatus)
        gstate = PyGILState_Ensure();
}

PyLockHelper::~PyLockHelper(){

    if( gstate )
        PyGILState_Release(gstate);
}
