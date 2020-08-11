#pragma once
#include <force/fileConfiguer.h>
#include <string>
#include <yaml-cpp/yaml.h>
#include <vector>
using namespace std;
//using namespace hirop_force;
class fileConfiguer
{
public:
    fileConfiguer(const std::string &fileName);

    int getForceName(std::string &gripperName);

    int getPrivateParams(YAML::Node &yamlNode);
    static vector<string> splitStr(string str, string &pattern);

private:
    YAML::Node config;
};

