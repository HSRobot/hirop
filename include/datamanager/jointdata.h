#pragma once

#include "hdata.h"
#include <vector>

#include <boost/serialization/vector.hpp>

namespace hirop{

namespace data_manager{

class JointData : public HData{

public:
    JointData() : HData("joint"){}

    /**
     * @brief joints    关节数据
     */
    std::vector<double> joints;

private:
    /**
     *  声明需要被序列化的对象
     */
    START_NEED_SAVE_OR_LOAD()
    VAR_NEED_SAVE_OR_LOAD(joints);
    END_NEED_SAVE_OR_LOAD()
};

}
}
