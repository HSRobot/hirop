#pragma once

#include <iostream>
#include <vector>

namespace hirop{

namespace data_manager {

/**
 * @brief The DataUri class 用于描述Data的唯一标识符
 */
class DataUri{

public:

    /**
     * @brief DataUri   创建一个不带名称的uri
     */
    DataUri(){}

    /**
     * @brief DataUri   创建一个带名称的uri
     * @param name      uri的名称
     */
    DataUri(std::string name);

    /**
     * @brief setUriFromStr 从字符串中构建uri
     * @param uriStr        字符串
     * @return              0 成功 -1 失败
     */
    int setUriFromStr(std::string uriStr);

    /**
     * @brief toString    获取URI的字符串表示
     * @return
     */
    const std::string toStr();

    /**
     * @brief getName   获取URI的名称
     * @return          名称
     */
    std::string getName() const {return _name;}

    /**
     * @brief getFlags  获取URI的标识列表
     * @return          URI的标识列表
     */
    std::vector<std::string> getFlags() const {return _flags;}

    /**
     * @brief addFlags
     * @param flag
     */
    void addFlag(std::string flag);

private:

    /**
     * @brief _name     数据的名称
     */
    std::string _name;

    /**
     * @brief _flags    URI的路径列表
     */
    std::vector<std::string> _flags;
};

}
}
