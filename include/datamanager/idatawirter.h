#pragma once

#include "hdata.h"
#include "datauri.h"

namespace hirop{

namespace data_manager{

class IDataWriter{

public:

    /**
     * @brief loadData  加载数据
     * @param uri       数据的URI
     * @return          成功则返回被加载的数据 失败则返回NULL
     */
    virtual HData *loadData( DataUri &uri) = 0;

    /**
     * @brief saveData  保存数据
     * @param data      需要被保存的数据
     * @param uri       数据的URI
     * @return          0 成功 -1 失败
     */
    virtual int saveData(HData *data, DataUri &uri) = 0;

    /**
     * @brief deletData 删除指定数据
     * @param uri       数据的uri
     * @return          0 成功 -1 失败
     */
    virtual int deleteData(DataUri &uri) = 0;

    /**
     * @brief getAllDatas   获取指定uri下的所有数据
     * @param uri[in]           uri
     * @param datas[out]         数据
     * @return  0 成功 -1 失败
     */
    virtual int getAllDatas(DataUri &uri, std::vector<HData*> &datas) = 0;

    /**
     * @brief listUri       获取指定uri下的所有uri
     * @param uri[in]       指定的uri
     * @param uris[out]     所有的uri
     * @return              0 成功 -1 失败
     */
    virtual int listUri(DataUri &uri, std::vector<DataUri> &uris) = 0;

};

}
}
