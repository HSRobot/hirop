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

    if(raw == "")
        return NULL;

    HData * data = HData::fromBuffer(raw);

    return data;
}

std::string FileDataWriter::loadRawData(DataUri &uri){

    const std::string uriStr = uri.toStr();
    std::string dataName = uri.getName();
    std::string fileName = _basePath + "/" + uriStr + dataName;

    std::ifstream ifile(fileName.c_str());

    if(ifile.fail()){
        return "";
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

    std::string uriStr = uri.toStr();
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

int FileDataWriter::deleteData(DataUri &uri){

    std::string uriStr = uri.toStr();
    std::string name = uri.getName();
    std::string path = _basePath + "/" + uriStr + name;

    if( !boost::filesystem::exists(path))
        return -1;

    return boost::filesystem::remove(path);
}

int FileDataWriter::getAllDatas(DataUri &uri, std::vector<HData *> &datas){

    std::vector<DataUri> uris;

    listUri(uri, uris);

    for(int i = 0; i < uris.size(); i++){
        HData *data = loadData(uris[i]);
        if(data != NULL)
            datas.push_back(data);
    }

    return 0;
}

int FileDataWriter::listUri(DataUri &uri, std::vector<DataUri> &uris){

    std::string uriStr = uri.toStr();
    std::string pathStr = _basePath + "/" + uriStr;

    boost::filesystem::directory_iterator end;

    if(!boost::filesystem::exists(pathStr))
        return -1;

    /**
     *  变量目录下的所有文件
     */
    for(boost::filesystem::directory_iterator pos(pathStr); pos != end; ++pos){

        if(boost::filesystem::is_directory(*pos)){
            continue;
        }

        /**
         * 通过构建uri的方式来获取data 该方法并不完善
         */
        DataUri tmpUri(pos->path().filename().string());
        tmpUri.setUriFromStr(uriStr);

        uris.push_back(tmpUri);
    }

    return 0;
}
