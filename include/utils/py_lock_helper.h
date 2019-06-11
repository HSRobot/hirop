#ifndef __PY_LOCK_HELPER_H__
#define __PY_LOCK_HELPER_H__

#include <Python.h>

class PyLockHelper{

public:
    PyLockHelper();

    ~PyLockHelper();

private:
    PyGILState_STATE gstate;

    PyThreadState *_save;

    int nStatus;
};

#endif
