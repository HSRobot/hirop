#include "vision/py_loader.h"
#include "vision/py_base_detector.h"
#include "utils/idebug.h"
#include "utils/py_lock_helper.h"
#include "utils/fs_helper.h"

#include <Python.h>

using namespace hirop_vision;

PyLoader* PyLoader::instance = NULL;

std::string PyLoader::PATH = "./";

ITrainer *PyLoader::loadTrainer(std::string trainerName){
    return NULL;
}

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

IDetector* PyLoader::loadDetector(std::string detectorName){

    PyLockHelper lock;

    PyObject* pModule = PyImport_ImportModule((detectorName + "Detector").c_str());

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

    PyObject* pDetectorClass = PyDict_GetItemString(pDict, (detectorName + "Detector").c_str());
    if( !pDict ){
        IErrorPrint("%s", "load python module class failed");
        PyErr_Print();
        return NULL;
    }

    PyBaseDetector *detector = new PyBaseDetector(pDetectorClass, (detectorName + "Detector"));

    //@todo free module and pDict

    Py_DECREF(pDetectorClass);
    Py_DECREF(pDict);
    Py_DECREF(pModule);

    return detector;
}

void PyLoader::getDetectorList(std::vector<std::string> &detectorList){

    std::vector<std::string> paths;
    paths.push_back(PATH);

    return FSHelper::filterFilesByRegx(paths, detectorList, PY_DETECTOR_REGEX);
}

int PyLoader::initSysPath(){

    const char *envPath;
    envPath = getenv("VISION_PYTHON_PLUGIN_PATH");
 
    if(envPath != NULL){
        PATH = envPath;
    }

    std::string chdir_cmd = std::string("sys.path.append(\"") + PATH + "\")";
    const char *cstr_cmd = chdir_cmd.c_str();

    PyRun_SimpleString("import sys");
    PyRun_SimpleString(cstr_cmd);
}

void PyLoader::getTrainerList(std::vector<std::string> &trainerList){
    return;
}
