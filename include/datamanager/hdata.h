#pragma once

#include <iostream>
#include <sstream>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/export.hpp>

namespace hirop{

namespace data_manager{

#define START_NEED_SAVE_OR_LOAD() \
    template <typename Archive> \
    void serialize(Archive &ar, const unsigned int version){ \
    ar & boost::serialization::base_object<HData>(*this);

#define END_NEED_SAVE_OR_LOAD() }\
    private:    \
        friend class boost::serialization::access;

#define VAR_NEED_SAVE_OR_LOAD(var) ar & var


/**
 * @brief The BaseData class   所有数据类型都需要实现的接口类
 */
class HData{

public:

    /**
     * @brief HData 构造函数
     */
    HData(std::string type) : _type(type){}

    /**
     * @brief HData 无参数构造函数
     */
    HData(){}

    /**
     *  析构函数
     */
    virtual ~HData(){}

    /**
     * @brief toBuffer  将对象序列化成字节
     * @return          对象的字节数据
     */
    const std::string toBuffer();

    /**
     * @brief fromBuffer    从对象序列中获取对象
     * @return              获取到的对象
     */
    static HData* fromBuffer(std::string buffer);

    /**
     * @brief getType   获取对象的类别
     * @return          对象的类别
     */
    std::string getType() const {return _type;}

private:

    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        ar & _type;
    }


private:
    friend class boost::serialization::access;
    /**
     * @brief type      保存对象的类别
     */
    std::string _type;
};

}
}
