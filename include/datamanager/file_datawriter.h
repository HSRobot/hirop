#pragma once

#include "idatawirter.h"

namespace hirop{

namespace data_manager{

class FileDataWriter : public IDataWriter{

public:

    /**
     * @brief FileDataWriter
     * @param basePath          数据将会在该文件夹下按规律存储
     */
    FileDataWriter(std::string basePath) : _basePath(basePath){}

    /**
     * @brief FileDataWriter    构造函数
     */
    FileDataWriter();

    /**
     * @brief FileDataWriter    析构函数
     */
    ~FileDataWriter();

    /**
     * @brief loadData  加载数据
     * @param uri       数据的URI
     * @return          成功则返回被加载的数据 失败则返回NULL
     */
    HData *loadData(DataUri &uri);

    /**
     * @brief loadRawData   加载对象的序列化数据
     * @param uri           对象的URI
     * @return              对象的序列化数据
     */
    std::string loadRawData(DataUri &uri);

    /**
     * @brief saveData  保存数据
     * @param data      需要被保存的数据
     * @param uri       数据的URI
     * @return          0 成功 -1 失败
     */
    int saveData(HData *data, DataUri &uri);

    /**
     * @brief saveRawData   保存对象的序列化数据
     * @param raw           需要保存的序列化数据
     * @param uri           数据的URI
     * @return              0 成功 -1 失败
     */
    int saveRawData(std::string raw, DataUri &uri);

    /**
     * @brief deleteData    删除指定数据
     * @param uri           数据的uri
     * @return              0 成功 -1 失败
     */
    int deleteData(DataUri &uri);

    /**
     * @brief getAllDatas   获取指定uri路径下的所有data
     * @param uri[in]
     * @param datas[out]
     * @return      0 成功 -1 失败
     */
    int getAllDatas(DataUri &uri, std::vector<HData *> &datas);


    int listUri(DataUri &uri, std::vector<DataUri> &uris);

private:
    /**
     * @brief _basePath     搜索的根路径
     */
    std::string _basePath;

};

}

}
