#ifndef __HPLUGIN_H__
#define __HPLUGIN_H__

#include "hpluginloader.h"

#include <iostream>
#include "hlibload.h"

class HPlugin{

public:
    /**
     * @brief       构造函数
     * @param[in] handle 动态库的句柄
     * @return      0
     */
    HPlugin(void *handle, std::string name, std::string version);

    /**
     * @brief       返回插件的名称
     * @return      插件的名称
     */
    std::string getName();

    /**
     * @brief       返回插件的版本号
     * @return      插件的版本号
     */
    std::string getVersion();

    /**
     * @brief       实例化一个插件
     * 在实例化前，会检查对象类型是否匹配。
     * @return      插件实例的指针
     */
    template <typename T>
    T *instance(){

        /**
         * 获取接口的IID
         */
        const char *interFaceid;
        interFaceid = H_GET_INTERFACE_ID(T);

        /**
         * 获取插件的IID
         */
        const char *pluginIid;
        GETSTR_PTR getiid_f;

        getiid_f = (GETSTR_PTR)loadFun(mLibHandle, GETIID_FUNNAME(pluginName.c_str()));
        if(!getiid_f){
            std::cout << "[error] can't get plugin iid" << std::endl;
            return NULL;
        }

        pluginIid = getiid_f();

        if(std::string(pluginIid) != std::string(interFaceid)){
            std::cout << "无法实例化插件: 你正在将一个插件实例化为其他类型的定义" << std::endl;
            return NULL;
        }

        if(!createFun){
            std::cout << "[error] can't instance plugin: instacne function was NULL" << std::endl;
            return NULL;
        }

        return (T*)createFun();
    }

    /**
     * @brief       获取错误信息
     * @return      错误信息
     */
    std::string errorString();

public:
    /**
     * @brief mLibHandle 当前插件的动态库句柄
     */
    void *mLibHandle;

private:
    /**
     * @brief isSingle  当前插件是否是单例插件
     */
    bool *isSingle;

    /**
     * @brief pluginName  当前插件的名称
     */
    std::string pluginName;

    /**
     * @brief pluginVer    当前插件的版本
     */
    std::string pluginVer;

    /**
     * @brief errorStr      当前错误信息
     */
    std::string errorStr;

    /**
     * @brief createFun     获取插件实例对象的函数指针
     */
    VOIDPTR_PTR createFun;
};

#endif
