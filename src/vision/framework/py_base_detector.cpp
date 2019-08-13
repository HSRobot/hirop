#pragma GCC diagnostic ignored "-Wconversion-null"

#include "vision/py_base_detector.h"
#include "utils/idebug.h"
#include "utils/py_lock_helper.h"

#include <stdlib.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <numpy/arrayobject.h>

using namespace hirop_vision;

PyBaseDetector::PyBaseDetector(PyObject *pClass, std::string detectorName){

    this->pClass = pClass;
    this->name = detectorName;

    if(!pClass){
        IErrorPrint("%s", "Python instance load failed: the class was null");
        return;
    }
    pClassInstance = PyObject_CallObject(pClass, NULL);
    if(!pClassInstance)
        IErrorPrint("%s", "Python Detector can't intance");

    entityType = PYTHON;
    initNump();
}

PyBaseDetector::~PyBaseDetector(){

    if(pClassInstance)
        Py_DECREF(pClassInstance);

}

int PyBaseDetector::initNump(){
    import_array();
}

int PyBaseDetector::loadData(const std::string path, const std::string objectName){

    PyLockHelper lock;

    if(!pClassInstance){
        IErrorPrint("%s", "Python detector load data failed: the class not instance");
        PyErr_Print();
        return -1;
    }

    PyObject* ret =  PyObject_CallMethod(pClassInstance, "loadData", "ss",  path.c_str(), objectName.c_str());

    if(!ret){
        PyErr_Print();
        return -1;
    }

    return 0;
}

int PyBaseDetector::detection(){

    PyLockHelper lock;

    if(!pClassInstance){
        IErrorPrint("%s", "Python detector start detetion error: the class not instance");
        return -1;
    }

    PyObject *ret = PyObject_CallMethod(pClassInstance, "detection","");
    if(!ret){
        PyErr_Print();
        return -1;
    }
    return 0;
}

void PyBaseDetector::setColorImg(const cv::Mat &inputImg){

    PyLockHelper lock;

    int rows =  inputImg.rows;
    int cols = inputImg.cols;
    int channels = inputImg.channels();

    IDebug("rows = %d, cols = %d, channels = %d", rows, cols, channels);

    //@todo 此处会造成内存泄露
    char * colorTmp= new char[rows * cols * channels];
    memcpy(colorTmp, inputImg.ptr<uchar>(0), rows * cols * channels);

    IDebug("memcpy mat finish");

    npy_intp Dims[3] = {rows, cols, channels};
    PyObject *matObj = PyArray_SimpleNewFromData(3, Dims, NPY_UBYTE, colorTmp);

    IDebug("PyArray_SimpleNewFromData finish");

    PyObject *ret = PyObject_CallMethod(pClassInstance, "setColorImg", "O", matObj);
    if(!ret)
        PyErr_Print();
}

void PyBaseDetector::setDepthImg(const cv::Mat &inputImg){

    PyLockHelper lock;
    int rows =  inputImg.rows;
    int cols = inputImg.cols;
    int channels = inputImg.channels();

    IDebug("rows = %d, cols = %d, channels = %d", rows, cols, channels);

    //@todo 此处会造成内存泄露
    char * depthTmp= new char[rows * cols * channels];
    memcpy(depthTmp, inputImg.ptr<uchar>(0), rows * cols * channels);

    npy_intp Dims[3] = {rows, cols, channels};
    PyObject *matObj = PyArray_SimpleNewFromData(3, Dims, NPY_UBYTE, depthTmp);

    PyObject *ret = PyObject_CallMethod(pClassInstance, "setDepthImg", "O", matObj);

    if(!ret)
        PyErr_Print();
}

int PyBaseDetector::getResult(std::vector<pose> &poses){
    PyLockHelper lock;

    if(!pClassInstance){
        IErrorPrint("%s", "Python detector getResult: the class not instance");
        return -1;
    }

    PyObject* ret =  PyObject_CallMethod(pClassInstance, "getResult", "");

    if(!ret)
        PyErr_Print();

    PyObject *R_item = PyList_GetItem(ret, 0);
    PyObject *T_item = PyList_GetItem(ret, 1);

    cv::Mat_<float> R_mat(3,3);

    double R[9];

    {
        PyArrayObject *array = (PyArrayObject *) R_item;
        int rows = array->dimensions[0], columns = array->dimensions[1];
        std::cout << "Rows = "  << rows << "columns ="<<  columns <<std::endl;
        for( int Index_m = 0; Index_m < rows; Index_m++){
            for(int Index_n = 0; Index_n < columns; Index_n++){
                R[Index_m * 3 + Index_n] = *(float *)(array->data + Index_m * array->strides[0] + Index_n * array->strides[1]);
            }
        }
        R_mat << R[0],R[1],R[2],R[3],R[4],R[5],R[6],R[7],R[8];
    }

    double T[3];
    {
        PyArrayObject *array = (PyArrayObject *) T_item;
        int rows = array->dimensions[0], columns = array->dimensions[1];
        for( int Index_m = 0; Index_m < rows; Index_m++){
            for(int Index_n = 0; Index_n < columns; Index_n++){
                T[Index_m] = *(float *)(array->data + Index_m * array->strides[0] + Index_n * array->strides[1]);
            }
        }
    }

    pose p;

    Eigen::Matrix3d t_R;
    cv::cv2eigen((cv::Matx33d)R_mat, t_R);
    Eigen::Quaterniond q(t_R);
    Eigen::Vector4d q_tmp = q.coeffs();

    p.quaternion.x = q_tmp[0];
    p.quaternion.y = q_tmp[1];
    p.quaternion.z = q_tmp[2];
    p.quaternion.w = q_tmp[3];

    p.position.x = T[0];
    p.position.y = T[1];
    p.position.z = T[2];

    poses.push_back(p);

    return 0;
}

int PyBaseDetector::getName(std::string &name){
    name = this->name;
    return 0;
}

ENTITY_TYPE PyBaseDetector::getEntityType(){
    return entityType;
}

int PyBaseDetector::isMultiDetector(){
    //    PyLockHelper lock;

    //    PyObject *ret = PyObject_CallMethod(pClassInstance, "isMultiDetector", "OO", pClassInstance, matObj);

    //    if(!ret)
    //        PyErr_Print();

    return 0;
}

int PyBaseDetector::parseConfig(const YAML::Node &node){
    return 0;
}
