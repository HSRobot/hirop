#include <datamanager/file_datawriter.h>
#include <boost/filesystem.hpp>
#include <fstream>

using namespace hirop::data_manager;

FileDataWriter::~FileDataWriter(){

}

FileDataWriter::FileDataWriter(){
    std::string homePath = getenv("HOME");
    _basePath = homePath + "/" +  ".hirop/data/";
}

HData* FileDataWriter::loadData(DataUri &uri) {

    std::string raw = loadRawData(uri);

    HData * data = HData::fromBuffer(raw);

    return data;
}

std::string FileDataWriter::loadRawData(DataUri &uri){

    const std::string uriStr = uri.toStr();
    std::string dataName = uri.getName();
    std::string fileName = _basePath + "/" + uriStr + dataName;

    std::ifstream ifile(fileName.c_str());

    if(ifile.fail()){
        return NULL;
    }

    std::stringstream dataStr;

    dataStr << ifile.rdbuf();

    return dataStr.str();
}

int FileDataWriter::saveData(HData *data, DataUri &uri){
    std::string raw = data->toBuffer();
    return saveRawData(raw, uri);
}

int FileDataWriter::saveRawData(std::string raw, DataUri &uri){

    const std::string uriStr = uri.toStr();
    std::string dataName = uri.getName();

    std::string path = _basePath + "/" + uriStr;

    /**
     *  如果路径不存在，则创建相关目录
     */
    if( !boost::filesystem::exists(path))
        boost::filesystem::create_directories(path);

    std::ofstream file((path + dataName).c_str());

    file << raw;

    file.close();

    return 0;
}
