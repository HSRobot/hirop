#include "vision/py_loader.h"
#include "vision/py_base_detector.h"
#include "utils/idebug.h"
#include "utils/py_lock_helper.h"

#include <Python.h>

using namespace hirop_vision;

PyLoader* PyLoader::instance = NULL;

const std::string PyLoader::PATH = "/home/fshs/KongWork/YOLO6d";

PyLoader* PyLoader::getPyLoader(){
    if(instance)
        return instance;

    instance = new PyLoader();
    return instance;
}

PyLoader::PyLoader(){
    Py_Initialize();
    initSysPath();

    PyEval_InitThreads();
    int nInit = PyEval_ThreadsInitialized();  //检测线程支持是否开启成功
    if(!nInit){
        IErrorPrint("%s", "PyLoader init threads failed");
    }
    PyEval_ReleaseThread(PyThreadState_Get());
}

IDetector* PyLoader::loadDetector(const std::string &detectorName){

    PyLockHelper lock;

    PyObject* pModule = PyImport_ImportModule(detectorName.c_str());

    if( !pModule ){
        IErrorPrint("%s", "load python module failed");
        PyErr_Print();
        return NULL;
    }

    PyObject* pDict = PyModule_GetDict(pModule);
    if( !pDict ){
        IErrorPrint("%s", "load python module dict failed");
        PyErr_Print();
        return NULL;
    }

    PyObject* pDetectorClass = PyDict_GetItemString(pDict, detectorName.c_str());
    if( !pDict ){
        IErrorPrint("%s", "load python module class failed");
        PyErr_Print();
        return NULL;
    }

    PyBaseDetector *detector = new PyBaseDetector(pDetectorClass, detectorName);

    //@todo free module and pDict

    Py_DECREF(pDetectorClass);
    Py_DECREF(pDict);
    Py_DECREF(pModule);

    return detector;
}

int PyLoader::initSysPath(){

    std::string chdir_cmd = std::string("sys.path.append(\"") + PATH + "\")";
    const char *cstr_cmd = chdir_cmd.c_str();

    PyRun_SimpleString("import sys");
    PyRun_SimpleString(cstr_cmd);
}
