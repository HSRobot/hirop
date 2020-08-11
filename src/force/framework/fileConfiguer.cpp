#include <force/fileConfiguer.h>
#include <utils/idebug.h>
#include <iostream>
//using namespace hirop_force;

fileConfiguer::fileConfiguer(const std::string &fileName)
{
    try{
         std::cout << fileName<<std::endl;
         config = YAML::LoadFile(fileName);
    }catch(std::exception e){
        IErrorPrint("Load configure file error");
    }
    if(config.IsNull())
    {
        IErrorPrint("YAML is Null()");
    }
}

int fileConfiguer::getForceName(std::string &gripperName){

    if(!config["forceName"]){
        IErrorPrint("Get gripper name error:  No gripperName node");
        return -1;
    }
    gripperName = config["forceName"].as<std::string>();
//    configDebug("Get gripperName: %s", gripperName.c_str());

    return 0;
}

int fileConfiguer::getPrivateParams(YAML::Node &yamlNode)
{
    assert(config.IsNull() == false);
    if(!config["parameters"]){
        IErrorPrint("parameters 无私有参数");
        return -1;
    }
    yamlNode = config;

    return 0;
}

vector<string> fileConfiguer::splitStr(string str, string &pattern)
{
    {
        string::size_type pos;
        vector<string> result;
        str += pattern;
        size_t size = str.size();
        for(size_t i = 0; i < size; i++)
        {
            pos = str.find(pattern,i);
            if(pos < size)
            {
                string s = str.substr(i,pos-i);
                result.push_back(s);
                i = pos + pattern.size() - 1;
            }
        }
        return result;
    }
}


