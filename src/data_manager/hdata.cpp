#include <datamanager/hdata.h>
#include <sstream>

using namespace hirop::data_manager;

const std::string HData::toBuffer(){

    std::stringstream os;
    boost::archive::binary_oarchive oa(os);

    HData* data = this;

    oa << data;

    return os.str();
}

HData* HData::fromBuffer(std::string buffer){

    std::stringstream is(buffer);

    boost::archive::binary_iarchive ia(is);

    HData *data;

    ia >> data;

    return data;
}

BOOST_SERIALIZATION_ASSUME_ABSTRACT(HData)
