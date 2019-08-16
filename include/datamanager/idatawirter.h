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

};

}
}
