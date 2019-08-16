#include <datamanager/jointdata.h>
#include <vector>
#include <datamanager/file_datawriter.h>

using namespace hirop::data_manager;

int main(){

    int mode;

    std::cout << "模式选择：1，序列化测试； 2，写测试； 3，读测试" << std::endl;
    std::cin >> mode;

    if(mode == 1){

        JointData *jdata = new JointData();

        std::vector<float> joints;
        joints.push_back(0.1);
        joints.push_back(0.2);
        joints.push_back(0.3);
        joints.push_back(0.4);
        jdata->joints = joints;

        /**
     * @brief tmp   将对象序列化至string中
     */
        std::string tmp = jdata->toBuffer();

        /**
     * @brief data  从string中将对象反序列化
     */
        JointData *data = (JointData *)HData::fromBuffer(tmp);
        std::cout << "data.type = " << data->getType() << std::endl;

        JointData *jdata_ptr = (JointData *)data;
        std::cout << "data.joints = " << jdata_ptr->joints[0] << " || " << jdata_ptr->joints[1]<< std::endl;
    }else if(mode == 2){


        JointData jointData;

        std::vector<float> joints;
        joints.push_back(0.1);
        joints.push_back(0.2);
        joints.push_back(0.3);
        joints.push_back(0.4);

        jointData.joints = joints;

        FileDataWriter writer;

        DataUri uri("joint1");
        uri.addFlag("TEST");
        writer.saveData(&jointData, uri);

    }else if(mode == 3){

        FileDataWriter writer;

        DataUri uri("joint1");
        uri.addFlag("TEST");

        JointData *data = (JointData *)writer.loadData(uri);
        std::cout << "data.type = " << data->getType() << std::endl;
        std::cout << "data.joints = " << data->joints[0] << " || " << data->joints[1]<< std::endl;

    }

}
