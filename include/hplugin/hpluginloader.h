#ifndef __HPLUGINLOADER_H__
#define __HPLUGINLOADER_H__

#include <vector>
#include <map>

/**
 * @brief HPlugin 由于头文件互相包含，需要先声明该类
 */
class HPlugin;

/**
 * @brief GETSTR_PTR 返回字符串的函数原型
 */
typedef const char* (*GETSTR_PTR)();

/**
 * @brief VOIDPTR_PTR 返回void指针的函数原型
 */
typedef void* (*VOIDPTR_PTR)();

/**
 * @brief GETNAME_FUNNAME 获取返回插件名称的函数名称
 */
#define GETNAME_FUNNAME(clasName)\
    (std::string("__getname") + clasName).c_str()

/**
 * @brief GETNAME_FUNNAME 获取返回插件版本的函数名称
 */
#define GETVER_FUNNAME(clasName)\
    (std::string("__getver") + clasName).c_str()

/**
 * @brief GETNAME_FUNNAME 获取返回插件IID的函数名称
 */
#define GETIID_FUNNAME(clasName)\
    (std::string("__getiid") + clasName).c_str()

/**
 * @brief GETNAME_FUNNAME 获取返回插件实例的函数名称
 */
#define CREATE_FUNNAME(clasName)\
    (std::string("__create") + clasName).c_str()

/**
 * @brief 模板特化函数，目的在于可以构建多种重名函数
 */
template <typename T>
const char *__hplugin_get_iid();

/**
 * @brief H_DECLARE_INTERFACE 定义插件的接口，在插件的头文件中使用，传递接口类以及其ID
 */
#define H_DECLARE_INTERFACE(Interface, Iid)\
    template<> inline const char *__hplugin_get_iid<Interface>(){ \
    return Iid; \
    }

/**
 * @brief H_DECLARE_PLUGIN 声明一个插件，表明该文件实现了一个插件，其插件实现的接口为Interface
 */
#define H_DECLARE_PLUGIN(Interface) \
    extern "C"{ \
    static const char *__plugin_iid = __hplugin_get_iid<Interface>(); \
    }

/**
 * @brief H_EXPORT_PLUGIN 导出一个插件，导出插件具体的类、名称和版本信息
 */
#define H_EXPORT_PLUGIN(className, name, version)\
    extern "C" { \
    void* __create##className(){ \
    return new className;       \
    }\
    char *__getname##className(){ \
    return name;    \
    }   \
    char *__getver##className(){ \
    return version; \
    } \
    const char *__getiid##className(){ \
    return __plugin_iid; \
    } \
    }

/**
 * @brief  H_GET_INTERFACE_ID 获取接口的IID
 */
#define H_GET_INTERFACE_ID(Interface) \
    __hplugin_get_iid<Interface>();


#include "hplugin.h"

typedef std::map<std::string, HPlugin *> HPluginList;

/**
 * @brief       插件加载器，单例模式。
 * @author      XuKunLin
 * @date        2019-04-23
 */
class HPluginLoader{

public:
    /**
     * @brief       获取插件加载器
     * @return      插件加载器指针
     */
    static HPluginLoader *getLoader();

    /**
     * @brief       析构函数
     */
    ~HPluginLoader();

    /**
     * @brief       列出当前插件系统已加载的插件
     * @return      当前已加载插件vector
     */
    HPluginList list();

    /**
     * @brief       设置插件系统动态库搜索路径
     * @param[in] path，路径
     * @return  0 成功 -1 失败
     */
    int setPath(const std::string &path);

    /**
     * @brief       卸载插件
     * @param[in] plugin，需要卸载的插件指针
     * @return      0 卸载成功 -1 卸载失败
     */
    int unload(HPlugin *plugin);

    /**
     * @brief       加载插件
     * @param[in] name，需要加载的插件名称
     * @return      插件指针
     * @todo 1，支持Python的加载
     */
    HPlugin* load(const std::string &name /*TYPE*/);

    /**
     * @brief       返回错误信息
     * @return      错误信息
     */
    std::string errorString();

private:
    /**
     * @brief       构造函数
     */
    HPluginLoader();

    /**
     * @brief       检查当前插件是否为指定接口类型的插件
     * @param
     * @return      0 是 -1 不是
     */
    int checkType();

private:
    /**
     * @brief mLoaderPtr    单例指针
     */
    static HPluginLoader *mLoaderPtr;

    /**
     * @brief mPluginList   保存当前系统中已加载的插件
     */
    HPluginList mPluginList;

    /**
     * @brief mPath  保存当前系统搜索插件的路径
     */
    std::string mPath;

};

#endif
