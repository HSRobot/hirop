#include <datamanager/datauri.h>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

using namespace hirop::data_manager;

DataUri::DataUri(std::string name){
    _name = name;
}

const std::string DataUri::toStr(){

    std::string uriStr;

    for(int i = 0; i < _flags.size(); i++)
        uriStr = uriStr + _flags[i] + "/";

    return uriStr;
}

void DataUri::addFlag(std::string flag){
    _flags.push_back(flag);
}

int DataUri::setUriFromStr(std::string uriStr){

    if(uriStr == "")
        return -1;

    std::vector<std::string> flags;
    boost::split(flags, uriStr, boost::is_any_of("/"));

    BOOST_FOREACH(std::string flag, flags){
        _flags.push_back(flag);
    }

    return 0;

}

