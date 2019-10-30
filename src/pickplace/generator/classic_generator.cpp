#include "classic_generator.h"

ClassicGenerator::ClassicGenerator()
{

}

ClassicGenerator::~ClassicGenerator()
{

}

int ClassicGenerator::parseConfig(YAML::Node& yamlNode)
{
    YAML::Node node,followPostureNode,grabPostureNode,grabOffsetNode,placePostureNode,placeOffsetNode;
    if(!yamlNode["parameters"]){
        IErrorPrint("无参数设置");
        return -1;
    }
    m_parm.sorting = yamlNode["parameters"]["sorting"].as<bool>();
    node = yamlNode["parameters"];
    if(node["followPosture"]){
        followPostureNode = node["followPosture"];
        m_parm.follow_R = node["followPosture"]["R"].as<bool>();
        m_parm.follow_P = node["followPosture"]["P"].as<bool>();
        m_parm.follow_Y = node["followPosture"]["Y"].as<bool>();
    }
    if(node["grabPosture"]){
        m_parm.dir_R = node["grabPosture"]["DIR_R"].as<double>();
        m_parm.dir_P = node["grabPosture"]["DIR_P"].as<double>();
        m_parm.dir_Y = node["grabPosture"]["DIR_Y"].as<double>();
    }
    if(node["grabOffset"]){
        m_parm.offset_x =   node["grabOffset"]["x"].as<double>();
        m_parm.offset_y =   node["grabOffset"]["y"].as<double>();
        m_parm.offset_z =   node["grabOffset"]["z"].as<double>();
    }
    if(node["placePosture"]){
        m_parm.p_dir_R = node["placePosture"]["DIR_R"].as<double>();
        m_parm.p_dir_P = node["placePosture"]["DIR_P"].as<double>();
        m_parm.p_dir_Y = node["placePosture"]["DIR_Y"].as<double>();
    }
    if(node["placeOffset"]){
        m_parm.p_offset_x =   node["placeOffset"]["x"].as<double>();
        m_parm.p_offset_y =   node["placeOffset"]["y"].as<double>();
        m_parm.p_offset_z =   node["placeOffset"]["z"].as<double>();
    }

#ifdef _COUT_
    std::cout<<
        "m_parm.sorting:"<<m_parm.sorting<<std::endl<<
        "m_parm.follow_R:"<<m_parm.follow_R<<std::endl<<
        "m_parm.follow_P:"<<m_parm.follow_P<<std::endl<<
        "m_parm.follow_Y:"<<m_parm.follow_Y<<std::endl<<
        "m_parm.dir_R:"<<m_parm.dir_R<<std::endl<<
        "m_parm.dir_P:"<<m_parm.dir_P<<std::endl<<
        "m_parm.dir_Y:"<<m_parm.dir_Y<<std::endl<<
        "m_parm.offset_x:"<<m_parm.offset_x<<std::endl<<
        "m_parm.offset_y:"<<m_parm.offset_y<<std::endl<<
        "m_parm.offset_z:"<<m_parm.offset_z<<std::endl<<
        "m_parm.p_dir_R:"<<m_parm.p_dir_R<<std::endl<<
        "m_parm.p_dir_P:"<<m_parm.p_dir_P<<std::endl<<
        "m_parm.p_dir_Y:"<<m_parm.p_dir_Y<<std::endl<<
        "m_parm.p_offset_x:"<<m_parm.p_offset_x<<std::endl<<
        "m_parm.p_offset_y:"<<m_parm.p_offset_y<<std::endl<<
        "m_parm.p_offset_z:"<<m_parm.p_offset_z<<std::endl;
#endif
    return 0;
}

int ClassicGenerator::setPickPose(PoseStamped pickObjPose)
{
    m_pickpose = pickObjPose;
    return 0;
}

int ClassicGenerator::setPlacePose(PoseStamped placeObjPose)
{
    m_placepose = placeObjPose;
    return 0;
}

int ClassicGenerator::genPickPose()
{
    euler origin_euler,cor_euler;
    Quaternion quat;

    quaternionToEuler(m_pickpose.pose.orientation, origin_euler);

#ifdef _COUT_
    std::cout<<"orientation.w:"<<m_pickpose.pose.orientation.w<<std::endl
             <<"orientation.x:"<<m_pickpose.pose.orientation.x<<std::endl
             <<"orientation.y:"<<m_pickpose.pose.orientation.y<<std::endl
             <<"orientation.x:"<<m_pickpose.pose.orientation.z<<std::endl;
    std::cout<<"con_euler_r:"<<origin_euler.roll<<std::endl
             <<"con_euler_p:"<<origin_euler.pitch<<std::endl
             <<"con_euler_y:"<<origin_euler.yaw<<std::endl;
#endif

    if(!m_parm.sorting){
        correctEuler(origin_euler, cor_euler);
        origin_euler = cor_euler;
    }

//    double mid_r = origin_euler.roll;
//    double mid_p = origin_euler.pitch;
//    double mid_y = origin_euler.yaw;

//    origin_euler.roll = mid_y;
//    origin_euler.yaw = mid_r;

    if(!m_parm.follow_R){
        origin_euler.roll = 0;
    }
    if(!m_parm.follow_P){
        origin_euler.pitch = 0;
    }
    if(!m_parm.follow_Y){
        origin_euler.yaw = 0;
    }

    origin_euler.roll += m_parm.dir_R;
    origin_euler.pitch += m_parm.dir_P;
    origin_euler.yaw += m_parm.dir_Y;

    eulerToQuaternion(origin_euler, quat);

#define _QUATPRO_
#ifdef _QUATPRO_
    Quaternion quatCon;

    quatConvect(quatCon);

#ifdef _COUT_
    std::cout<<"quatConvect.px,"
              <<"quatConvect.py,"
              <<"quatConvect.pz,"
              <<"quatConvect.qx,"
              <<"quatConvect.qy,"
              <<"quatConvect.qz,"
              <<"quatConvect.qw ="

              <<m_pickpose.pose.position.x<<" "
              <<m_pickpose.pose.position.y<<" "
              <<m_pickpose.pose.position.z<<" "
              <<quatCon.x <<" "
              <<quatCon.y<<" "
              <<quatCon.z<<" "
              <<quatCon.w <<std::endl;
#endif

#endif

    m_pickpose.pose.position.x += m_parm.offset_x;
    m_pickpose.pose.position.y += m_parm.offset_y;
    m_pickpose.pose.position.z += m_parm.offset_z;
    m_pickpose.pose.orientation = quatCon;

#ifdef _COUT_
    std::cout<<"orientation.w:"<<m_pickpose.pose.orientation.w<<std::endl
             <<"orientation.x:"<<m_pickpose.pose.orientation.x<<std::endl
             <<"orientation.y:"<<m_pickpose.pose.orientation.y<<std::endl
             <<"orientation.x:"<<m_pickpose.pose.orientation.z<<std::endl;
    std::cout<<"origin_euler_r:"<<origin_euler.roll<<std::endl
             <<"origin_euler_p:"<<origin_euler.pitch<<std::endl
             <<"origin_euler_y:"<<origin_euler.yaw<<std::endl;
#endif

#ifdef _COUT_
    std::cout<<"frame_id:"<<m_pickpose.frame_id<<std::endl
             <<"m_pickpose.pose.px:"<<m_pickpose.pose.position.x<<std::endl
             <<"m_pickpose.pose.py:"<<m_pickpose.pose.position.y<<std::endl
             <<"m_pickpose.pose.pz:"<<m_pickpose.pose.position.z<<std::endl
             <<"m_pickpose.pose.ox:"<<m_pickpose.pose.orientation.x<<std::endl
             <<"m_pickpose.pose.oy:"<<m_pickpose.pose.orientation.y<<std::endl
             <<"m_pickpose.pose.oz:"<<m_pickpose.pose.orientation.z<<std::endl
             <<"m_pickpose.pose.ow:"<<m_pickpose.pose.orientation.w<<std::endl;
#endif
    return 0;
}

int ClassicGenerator::genPlacePose()
{
    euler place_euler;
    Quaternion place_quat;

    quaternionToEuler(m_placepose.pose.orientation, place_euler);
    place_euler.roll += m_parm.p_dir_R;
    place_euler.pitch += m_parm.p_dir_P;
    place_euler.yaw += m_parm.p_dir_Y;

    eulerToQuaternion(place_euler, place_quat);

    m_placepose.pose.position.x += m_parm.p_offset_x;
    m_placepose.pose.position.y += m_parm.p_offset_y;
    m_placepose.pose.position.z += m_parm.p_offset_z;
    m_placepose.pose.orientation = place_quat;

#ifdef _COUT_
    std::cout<<"frame_id:"<<m_placepose.frame_id<<std::endl
              <<"m_placepose.pose.px:"<<m_placepose.pose.position.x<<std::endl
              <<"m_placepose.pose.py:"<<m_placepose.pose.position.y<<std::endl
              <<"m_placepose.pose.pz:"<<m_placepose.pose.position.z<<std::endl
              <<"m_placepose.pose.ox:"<<m_placepose.pose.orientation.x<<std::endl
              <<"m_placepose.pose.oy:"<<m_placepose.pose.orientation.y<<std::endl
              <<"m_placepose.pose.oz:"<<m_placepose.pose.orientation.z<<std::endl
              <<"m_placepose.pose.ow:"<<m_placepose.pose.orientation.w<<std::endl;
#endif

    return 0;
}

int ClassicGenerator::getResultPickPose(PoseStamped &pickPoses)
{
    pickPoses = m_pickpose;
    return 0;
}

int ClassicGenerator::getResultPlacePose(PoseStamped &placePoses)
{
    placePoses = m_placepose;
    return 0;
}

int ClassicGenerator::quaternionToEuler(Quaternion quat,euler& euler)
{
    tf::Quaternion quatt;
    quatt.setW(quat.w);
    quatt.setX(quat.x);
    quatt.setY(quat.y);
    quatt.setZ(quat.z);

    tf::Matrix3x3(quatt).getRPY(euler.roll,euler.pitch,euler.yaw);

    return 0;
}

int ClassicGenerator::eulerToQuaternion(euler euler, Quaternion& quat)
{

    tf::Quaternion tq;
    tq = tf::createQuaternionFromRPY(euler.roll,euler.pitch,euler.yaw);

    quat.w = tq.w();
    quat.x = tq.x();
    quat.y = tq.y();
    quat.z = tq.z();
    return 0;
}

int ClassicGenerator::quatPro(Quaternion quat1, Quaternion quat2, Quaternion &quat3)
{
    float w1, x1, y1, z1;
    float w2, x2, y2, z2;
    w2 = quat1.w;
    x2 = quat1.x;
    y2 = quat1.y;
    z2 = quat1.z;

    w1 = quat2.w;
    x1 = quat2.x;
    y1 = quat2.y;
    z1 = quat2.z;

    quat3.w = (w1*w2) - (x1*x2) - (y1*y2) - (z1*z2);
    quat3.x = (w1*x2) + (x1*w2) + (y1*z2) - (z1*y2);
    quat3.y = (w1*y2) - (x1*z2) + (y1*w2) + (z1*x2);
    quat3.z = (w1*z2) + (x1*y2) - (y1*x2) + (z1*w2);
    return 0;
}

int ClassicGenerator::quatFromVector(double vec[4], Quaternion &quat)
{
    double vx = vec[0];
    double vy = vec[1];
    double vz = vec[2];
    double tan = vec[3];

    double qw = cos(tan/2);
    double qx = sin(tan/2) * vx;
    double qy = sin(tan/2) * vy;
    double qz = sin(tan/2) * vz;

    quat.w = (float)qw;
    quat.x = (float)qx;
    quat.y = (float)qy;
    quat.z = (float)qz;

    return 0;

}

int ClassicGenerator::quatConvect(Quaternion &quatConvect)
{
    Quaternion quatOrigin, quatVerCx, quatVerCy, quatVerCz, quatConvectx, quatConvecty, quatConvectz;
    euler eulerP;

    quaternionToEuler(m_pickpose.pose.orientation, eulerP);

    quatOrigin.w = 1;
    quatOrigin.x = 0;
    quatOrigin.y = 0;
    quatOrigin.z = 0;

    double vecy[4] = {0, 1, 0, (m_parm.dir_P + eulerP.pitch)};
    quatFromVector(vecy, quatVerCy);
    quatPro(quatOrigin, quatVerCy, quatConvecty);

    double vecx[4] = {1, 0, 0, (m_parm.dir_R + eulerP.roll)};
    quatFromVector(vecx, quatVerCx);
    quatPro(quatConvecty, quatVerCx, quatConvectx);

    double vecz[4] = {0, 0, 1, (m_parm.dir_Y + eulerP.yaw)};
    quatFromVector(vecz, quatVerCz);
    quatPro(quatConvectx, quatVerCz, quatConvectz);

    quatConvect = quatConvectz;

}

int ClassicGenerator::correctEuler(euler origin,euler& out_euler)
{
#ifdef _COUT_
    std::cout<<"origin.r =" <<origin.roll <<std::endl;
    std::cout<<"origin.p =" <<origin.pitch <<std::endl;
    std::cout<<"origin.y =" <<origin.yaw <<std::endl;
#endif

    if(origin.pitch < -1.57)
        origin.pitch =origin.pitch + 3.14;
    if(origin.roll < -1.57)
        origin.roll =origin.roll + 3.14;
    if(origin.yaw < -1.57)
        origin.yaw =origin.yaw + 3.14;

    if(origin.pitch > 1.57)
        origin.pitch =origin.pitch - 3.14;
    if(origin.roll  > 1.57)
        origin.roll =origin.roll - 3.14;
    if(origin.yaw > 1.57)
        origin.yaw =origin.yaw - 3.14;

    out_euler.pitch = origin.pitch;
    out_euler.roll = origin.roll;
    out_euler.yaw = origin.yaw;

#ifdef _COUT_
    std::cout<<"out_euler.r =" <<out_euler.roll <<std::endl;
    std::cout<<"out_euler.p =" <<out_euler.pitch <<std::endl;
    std::cout<<"out_euler.y =" <<out_euler.yaw <<std::endl;
    return 0;
#endif
}

//template<typename T>
//T getParam(const YAML::Node& node,const std::string& name,const T& defaultValue)
//{
//    T v;
//    try {
//        v=node[name].as<T>();//读取参数
//            std::cout<<"Found parameter: "<<name<<",\tvalue: "<<v<<std::endl;//终端提示读取成功
//    } catch (std::exception e) {
//       //找不到该参数的话，将返回默认值
//            v=defaultValue;
//        std::cout<<"Cannot find parameter: "<<name<<",\tassigning  default: "<<v<<std::endl;
//    }
//    return v;
//}

int ClassicGenerator::stopGenerator()
{
    return 0;
}

H_EXPORT_PLUGIN(ClassicGenerator,  "ClassicGenerator",  "1.0")
