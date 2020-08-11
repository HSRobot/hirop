
#include <hsImpenderrForce.h>
#include <force/fileConfiguer.h>
//#define HIROP_NAME
#define VERSION  "1.0"

using namespace hirop_force;
hsImpenderrForce::hsImpenderrForce():pluginName("hsImpenderrForce")
{
    ts = 0.04;

    dXa_1 = new double[6]();
    Xa_1 = new double[6]();
    ddXa = new double[6]();
    Xa = new double[6]();

    for(int i = 0; i < 6; i++){
        direction[i] = 1;
    }

    for(int i = 0; i < 6; i++)
    {
        m_Mass[i] = 1;
        m_Damping[i] = 5;
        m_Stiffness[i] = 5;
    }
}

hsImpenderrForce::~hsImpenderrForce()
{
    delete dXa_1;
    delete Xa_1;
    delete ddXa;

}

int hsImpenderrForce::parseConfig(YAML::Node &node)
{
    this->node = node;
    try{
        string time = node["parameters"]["time_stamp"].as<string>();
        string direction = node["parameters"]["direction"].as<string>();
        string mass = node["parameters"]["Mass"].as<string>();
        string damping = node["parameters"]["Damping"].as<string>();
        string stiffness = node["parameters"]["Stiffness"].as<string>();

        string split = ",";
        vector<string> splitOutParam;
        splitOutParam = fileConfiguer::splitStr(direction, split);
        transformVecStr2Array(this->direction, 6, splitOutParam);

        splitOutParam = fileConfiguer::splitStr(mass, split);
        transformVecStr2Array(m_Mass, 6, splitOutParam);

        splitOutParam = fileConfiguer::splitStr(damping, split);
        transformVecStr2Array(m_Damping, 6, splitOutParam);

        splitOutParam = fileConfiguer::splitStr(stiffness, split);
        transformVecStr2Array(m_Stiffness, 6, splitOutParam);

        ts = std::atof(time.c_str())/1000.;


    }catch(YAML::Exception &e){
        std::cout << e.what()<<std::endl;
        return -1;
    }

    std::cout << "#INFO parseConfig ok "<<std::endl;


    return 0;
}

void hsImpenderrForce::setInputForceBias(std::vector<double> &value)
{
    assert(value.size() == 6);
    Force.swap(value);
}

void hsImpenderrForce::setInputRobotCartPose(std::vector<double> &value)
{
    assert(value.size() == 6);
    robotPose.swap(value);
}

int hsImpenderrForce::compute()
{
    assert(Force.size() == 6);
    assert(robotPose.size() == 6);

    static double aBias = 0.25;//0.38;
    static double vBias = 0.35; //0.45

    for (int i = 0; i < 6; i++)
    {
        if(direction[i] == 0)
        {
            Xa_1[i] = 0;
            continue;
        }

        ddXa[i] = ( Force[i] - m_Damping[i]*dXa_1[i] - m_Stiffness[i]*Xa_1[i] ) / m_Mass[i];
        //阻抗偏移加速度限制 0.3 0.5
        if(ddXa[i] > aBias)
            ddXa[i] = aBias;
        else if(ddXa[i] < -aBias)
            ddXa[i] = -aBias;

        dXa[i] = dXa_1[i] + ddXa[i] * ts;
        //阻抗偏移速度限制
        if(dXa[i] >vBias)
            dXa[i]  = vBias;
        else if(dXa[i] < -vBias)
            dXa[i]  = -vBias;

        Xa[i] = Xa_1[i] + dXa[i] * ts;

        //变量回归
        dXa_1[i] = dXa[i];
        Xa_1[i] = Xa[i];
        if(Xa_1[i] > 100000. ){
            exit(-1);
        }
    }
    return 0;
}

int hsImpenderrForce::getResult(std::vector<double> &pose)
{
    pose.resize(6);
    copy(&Xa_1[0], &Xa_1[6], pose.begin());
    assert(pose.size() == 6);

    return 0;
}

string hsImpenderrForce::getName()
{
    return pluginName;
}


int hsImpenderrForce::setDirection(int *direction)
{
    for(int i= 0; i< 6; i++)
    {
        this->direction[i] = *direction;
        direction++;
    }

}

int hsImpenderrForce::swithMode(int type)
{
    std::cout << "swithMode function reserve "<<std::endl;
}

void hsImpenderrForce::transformVecStr2Array(double *arr, int size, vector<string> &data)
{
    if( data.size() != size){
        std::cout << "transformVecStr2Array error DOUBLE  "<<std::endl;
        return ;
    }

    for(int i = 0; i< size; i++)
    {
        arr[i] = std::atof(data.at(i).c_str());
    }

}

void hsImpenderrForce::transformVecStr2Array(int *arr, int size, vector<string> &data)
{
    if( data.size() != size){
        std::cout << "transformVecStr2Array error INT  "<<std::endl;
        return ;
    }

    for(int i = 0; i< size; i++)
    {
        arr[i] = std::atoi(data.at(i).c_str());
    }
}

int hsImpenderrForce::printInfo()
{
    std::cout << "name : "<< pluginName<<std::endl;
}


H_EXPORT_PLUGIN(hsImpenderrForce, "hsImpenderrForce" , "1.0")
