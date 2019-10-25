#include "classic_actuator.h"

ClassicActuator::ClassicActuator()
{
    _init();
}

ClassicActuator::~ClassicActuator()
{
    _moveGroup = NULL;
    delete _moveGroup;
}

int ClassicActuator::parseConfig(YAML::Node& yamlNode)
{
    YAML::Node node;
    if(!yamlNode["parameters"]){
        std::cerr<<"无参数设置"<<std::endl;
        return -1;
    }
    node = yamlNode["parameters"];
    if(node["geometry"]){
        m_parm.geometry.type = node["geometry"]["type"].as<std::string>();
        m_parm.geometry.length = node["geometry"]["L"].as<double>();
        m_parm.geometry.width  = node["geometry"]["W"].as<double>();
        m_parm.geometry.height = node["geometry"]["H"].as<double>();
        m_parm.geometry.mesh = node["geometry"]["mesh"].as<std::string>();
        m_parm.geometry.table_link = node["geometry"]["talbe_link"].as<std::string>();
        m_parm.geometry.table_px = node["geometry"]["table_pose_x"].as<double>();
        m_parm.geometry.table_py = node["geometry"]["table_pose_y"].as<double>();
        m_parm.geometry.table_pz = node["geometry"]["table_pose_z"].as<double>();
    }
    if(node["moveitConfig"]){
        m_parm.moveitConfig.planner_id = node["moveitConfig"]["plannerId"].as<std::string>();
        m_parm.moveitConfig.velocity = node["moveitConfig"]["velocity"].as<double>();
        m_parm.moveitConfig.accelerated = node["moveitConfig"]["accelerated"].as<double>();
    }
    if(node["pickConfig"]){
        m_parm.pickConfig.direct = node["pickConfig"]["direct_pick"].as<bool>();
        m_parm.pickConfig.m_Roll = node["pickConfig"]["Roll"].as<std::vector<double> >();
        m_parm.pickConfig.m_Pitch = node["pickConfig"]["Pitch"].as<std::vector<double> >();
        m_parm.pickConfig.m_Yaw = node["pickConfig"]["Yaw"].as<std::vector<double> >();

        m_parm.pickConfig.pre_dist_min = node["pickConfig"]["pre_dist_min"].as<float>();
        m_parm.pickConfig.pre_dist_max = node["pickConfig"]["pre_dist_max"].as<float>();
        m_parm.pickConfig.pre_vect_x =   node["pickConfig"]["pre_vect_x"].as<double>();
        m_parm.pickConfig.pre_vect_y =   node["pickConfig"]["pre_vect_y"].as<double>();
        m_parm.pickConfig.pre_vect_z =   node["pickConfig"]["pre_vect_z"].as<double>();
        m_parm.pickConfig.back_dist_min =  node["pickConfig"]["back_dist_min"].as<double>();
        m_parm.pickConfig.back_dist_max =  node["pickConfig"]["back_dist_max"].as<double>();
        m_parm.pickConfig.back_vect_x =  node["pickConfig"]["back_vect_x"].as<double>();
        m_parm.pickConfig.back_vect_y =  node["pickConfig"]["back_vect_y"].as<double>();
        m_parm.pickConfig.back_vect_z =  node["pickConfig"]["back_vect_z"].as<double>();
    }
    if(node["placeConfig"]){
        m_parm.placeConfig.direct = node["placeConfig"]["direct_place"].as<bool>();
        m_parm.placeConfig.m_Roll = node["placeConfig"]["Roll"].as<std::vector<double> >();
        m_parm.placeConfig.m_Pitch = node["placeConfig"]["Pitch"].as<std::vector<double> >();
        m_parm.placeConfig.m_Yaw = node["placeConfig"]["Yaw"].as<std::vector<double> >();

        m_parm.placeConfig.pre_dist_min = node["placeConfig"]["pre_dist_min"].as<float>();
        m_parm.placeConfig.pre_dist_max = node["placeConfig"]["pre_dist_max"].as<float>();
        m_parm.placeConfig.pre_vect_x =   node["placeConfig"]["pre_vect_x"].as<double>();
        m_parm.placeConfig.pre_vect_y =   node["placeConfig"]["pre_vect_y"].as<double>();
        m_parm.placeConfig.pre_vect_z =   node["placeConfig"]["pre_vect_z"].as<double>();
        m_parm.placeConfig.back_dist_min =  node["placeConfig"]["back_dist_min"].as<double>();
        m_parm.placeConfig.back_dist_max =  node["placeConfig"]["back_dist_max"].as<double>();
        m_parm.placeConfig.back_vect_x =  node["placeConfig"]["back_vect_x"].as<double>();
        m_parm.placeConfig.back_vect_y =  node["placeConfig"]["back_vect_y"].as<double>();
        m_parm.placeConfig.back_vect_z =  node["placeConfig"]["back_vect_z"].as<double>();
    }
    if(node["gripperConfig"]){
        m_parm.gripperConfig.joint_name = node["gripperConfig"]["joint_name"].as<std::string>();
        m_parm.gripperConfig.open_position = node["gripperConfig"]["open_position"].as<double>();
        m_parm.gripperConfig.close_position = node["gripperConfig"]["close_position"].as<double>();
    }


    addBaseTable();

#ifdef _COUT_
    std::cout<<"m_parm.geometry.type:"<<m_parm.geometry.type<<std::endl<<
        "m_parm.geometry.length:"<<m_parm.geometry.length<<std::endl<<
        "m_parm.geometry.width:"<<m_parm.geometry.width<<std::endl<<
        "m_parm.geometry.height:"<<m_parm.geometry.height<<std::endl<<
        "m_parm.geometry.mesh:"<<m_parm.geometry.mesh<<std::endl<<
        "m_parm.geometry.table_link:"<<m_parm.geometry.table_link<<std::endl<<
        "m_parm.geometry.table_px:"<<m_parm.geometry.table_px<<std::endl<<
        "m_parm.geometry.table_py:"<<m_parm.geometry.table_py<<std::endl<<
        "m_parm.geometry.table_pz:"<<m_parm.geometry.table_pz<<std::endl<<

        "m_parm.moveitConfig.planner_id:"<<m_parm.moveitConfig.planner_id<<std::endl<<
        "m_parm.moveitConfig.velocity:"<<m_parm.moveitConfig.velocity<<std::endl<<
        "m_parm.moveitConfig.accelerated:"<<m_parm.moveitConfig.accelerated<<std::endl<<
        "m_parm.pickConfig.direct:"<<m_parm.pickConfig.direct<<std::endl<<
        "m_parm.pickConfig.pre_dist_min:"<<m_parm.pickConfig.pre_dist_min<<std::endl<<
        "m_parm.pickConfig.pre_dist_max:"<<m_parm.pickConfig.pre_dist_max<<std::endl<<
        "m_parm.pickConfig.pre_vect_x:"<<m_parm.pickConfig.pre_vect_x<<std::endl<<
        "m_parm.pickConfig.pre_vect_y:"<<m_parm.pickConfig.pre_vect_y<<std::endl<<
        "m_parm.pickConfig.pre_vect_z:"<<m_parm.pickConfig.pre_vect_z<<std::endl<<
        "m_parm.pickConfig.back_dist_min:"<<m_parm.pickConfig.back_dist_min<<std::endl<<
        "m_parm.pickConfig.back_dist_max:"<<m_parm.pickConfig.back_dist_max<<std::endl<<
        "m_parm.pickConfig.back_vect_x:"<<m_parm.pickConfig.back_vect_x<<std::endl<<
        "m_parm.pickConfig.back_vect_y:"<<m_parm.pickConfig.back_vect_y<<std::endl<<
        "m_parm.pickConfig.back_vect_z:"<<m_parm.pickConfig.back_vect_z<<std::endl<<


        "m_parm.placeConfig.direct:"<<m_parm.placeConfig.direct<<std::endl<<
        "m_parm.placeConfig.pre_dist_min:"<<m_parm.placeConfig.pre_dist_min<<std::endl<<
        "m_parm.placeConfig.pre_dist_max:"<<m_parm.placeConfig.pre_dist_max<<std::endl<<
        "m_parm.placeConfig.pre_vect_x:"<<m_parm.placeConfig.pre_vect_x<<std::endl<<
        "m_parm.placeConfig.pre_vect_y:"<<m_parm.placeConfig.pre_vect_y<<std::endl<<
        "m_parm.placeConfig.pre_vect_z:"<<m_parm.placeConfig.pre_vect_z<<std::endl<<
        "m_parm.placeConfig.back_vect_x:"<<m_parm.placeConfig.back_vect_x<<std::endl<<
        "m_parm.placeConfig.back_vect_y:"<<m_parm.placeConfig.back_vect_y<<std::endl<<
        "m_parm.placeConfig.back_vect_z:"<<m_parm.placeConfig.back_vect_z<<std::endl<<

        "m_parm.gripperConfig.joint_name:"<<m_parm.gripperConfig.joint_name<<std::endl<<
        "m_parm.gripperConfig.open_position:"<<m_parm.gripperConfig.open_position<<std::endl<<
        "m_parm.gripperConfig.close_position:"<<m_parm.gripperConfig.close_position<<std::endl;

//        "m_parm.pickConfig.m_Roll:"<<m_parm.pickConfig.m_Roll<<
//        "m_parm.pickConfig.m_Pitch:"<<m_parm.pickConfig.m_Pitch<<
#endif

    return 0;
}

void ClassicActuator::_init()
{
     loadMoveit();
     removeObject();
     _pickStopFlag = false;
     return;
}

int ClassicActuator::loadMoveit()
{
    _moveGroup = new MoveGroupInterface(MOVE_GROUP);
    _moveGroup->setNumPlanningAttempts(3);
    _moveGroup->setPlanningTime(3);
    return 0;
}

int ClassicActuator::setMoveitConfig()
{
    setPlannerId(m_parm.moveitConfig.planner_id);
    setPickplaceSpeed(m_parm.moveitConfig.velocity, m_parm.moveitConfig.accelerated);
}

int ClassicActuator::setPlannerId(std::string plannerId)
{
    if(_moveGroup == NULL)
        return -1;
    _moveGroup->setPlannerId(plannerId);
    return 0;
}

int ClassicActuator::setPickplaceSpeed(double velocity,double accelerated)
{
    if(_moveGroup == NULL)
        return -1;
    _moveGroup->setMaxAccelerationScalingFactor(accelerated);
    _moveGroup->setMaxVelocityScalingFactor(velocity);
    return 0;
}

int ClassicActuator::setPickPose(PoseStamped pickPos)
{
    m_pickPose.header.frame_id = pickPos.frame_id;
    m_pickPose.pose.position.x = pickPos.pose.position.x;
    m_pickPose.pose.position.y = pickPos.pose.position.y;
    m_pickPose.pose.position.z = pickPos.pose.position.z;

    m_pickPose.pose.orientation.w = pickPos.pose.orientation.w;
    m_pickPose.pose.orientation.x = pickPos.pose.orientation.x;
    m_pickPose.pose.orientation.y = pickPos.pose.orientation.y;
    m_pickPose.pose.orientation.z = pickPos.pose.orientation.z;

#ifdef _COUT_
    std::cout<<"_pickposes.px ="<<m_pickPose.pose.position.x<<std::endl
             <<",_pickposes.py ="<<m_pickPose.pose.position.y<<std::endl
             <<",_pickposes.pz ="<<m_pickPose.pose.position.z<<std::endl;
#endif

    return 0;
}

int ClassicActuator::setPlacePose(PoseStamped placePos)
{
    m_placePose.header.frame_id = placePos.frame_id;
    m_placePose.pose.position.x = placePos.pose.position.x;
    m_placePose.pose.position.y = placePos.pose.position.y;
    m_placePose.pose.position.z = placePos.pose.position.z;

    m_placePose.pose.orientation.w = placePos.pose.orientation.w;
    m_placePose.pose.orientation.x = placePos.pose.orientation.x;
    m_placePose.pose.orientation.y = placePos.pose.orientation.y;
    m_placePose.pose.orientation.z = placePos.pose.orientation.z;

#ifdef _COUT_
    std::cout<<"_placeposes.px ="<<m_placePose.pose.position.x<<std::endl
             <<"_placeposes.py ="<<m_placePose.pose.position.y<<std::endl
             <<"_placeposes.pz ="<<m_placePose.pose.position.z<<std::endl;
#endif

    return 0;
}

int ClassicActuator::showObject(PoseStamped object_pose)
{
    _boxPose.header.frame_id = object_pose.frame_id;
    _boxPose.pose.position.x = object_pose.pose.position.x;
    _boxPose.pose.position.y = object_pose.pose.position.y;
    _boxPose.pose.position.z = object_pose.pose.position.z;

    _boxPose.pose.orientation.w = object_pose.pose.orientation.w;
    _boxPose.pose.orientation.x = object_pose.pose.orientation.x;
    _boxPose.pose.orientation.y = object_pose.pose.orientation.y;
    _boxPose.pose.orientation.z = object_pose.pose.orientation.z;

#ifdef _COUT_
    std::cout<<"box_pose.x = " <<_boxPose.pose.position.x<<std::endl
             <<"box_pose.y = " <<_boxPose.pose.position.y<<std::endl
             <<"box_pose.z = " <<_boxPose.pose.position.z<<std::endl
             <<"box_pose.ox = " <<_boxPose.pose.orientation.x<<std::endl
             <<"box_pose.oy = " <<_boxPose.pose.orientation.y<<std::endl
             <<"box_pose.oz = " <<_boxPose.pose.orientation.z<<std::endl
             <<"box_pose.ow = " <<_boxPose.pose.orientation.w<<std::endl
             <<"box_pose_frame_id = " << _boxPose.header.frame_id<<std::endl;
#endif

    if(m_parm.geometry.mesh == "NULL"){
        addCollisionObject("object");
        addCollision();
    }
    else{
        showObjectMash(m_parm.geometry.mesh);
        sleep(2.0);
    }
    return 0;

}

int ClassicActuator::removeObject()
{
    dettachCollision("object");
    removeCollision();
    return 0;
}

int ClassicActuator::addCollisionObject(std::string object_id)
{
    shape_msgs::SolidPrimitive primitive;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = _boxPose.header.frame_id;
    collision_object.id = object_id;
    _objectIds.push_back(collision_object.id);
    if(m_parm.geometry.type == "BOX")
        primitive.type = primitive.BOX;
    else if(m_parm.geometry.type == "CYLINDER")
        primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = m_parm.geometry.length;
    primitive.dimensions[1] = m_parm.geometry.width;
    primitive.dimensions[2] = m_parm.geometry.height;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(_boxPose.pose);
    collision_object.operation = collision_object.ADD;

    _collisionObjects.push_back(collision_object);

    return 0;
}

int ClassicActuator::addBaseTable()
{
    shape_msgs::SolidPrimitive primitive;
    moveit_msgs::CollisionObject collision_object;
    geometry_msgs::Pose base_pose;
    std::vector<moveit_msgs::CollisionObject> ob;
    collision_object.header.frame_id = m_parm.geometry.table_link;
    collision_object.id = "base_table";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 3;
    primitive.dimensions[1] = 3;
    primitive.dimensions[2] = 0.025;
    collision_object.primitives.push_back(primitive);
    base_pose.position.x = m_parm.geometry.table_px;
    base_pose.position.y = m_parm.geometry.table_py;
    base_pose.position.z = m_parm.geometry.table_pz;
    base_pose.orientation.w = 1;
    collision_object.primitive_poses.push_back(base_pose);
    collision_object.operation = collision_object.ADD;
    ob.push_back(collision_object);
    _planScene.addCollisionObjects(ob);
    IDebug("add base_table");
    sleep(1.0);
    return 0;
}

int ClassicActuator::addCollision()
{
    _planScene.addCollisionObjects(_collisionObjects);
    _collisionObjects.clear();
    IDebug("add_collision");
    return 0;
}

int ClassicActuator::removeCollision()
{
    _planScene.removeCollisionObjects(_objectIds);
    _objectIds.clear();
    return 0;
}

int ClassicActuator::attachCollision(std::string obj)
{
    _moveGroup->attachObject(obj);
    return 0;
}

int ClassicActuator::dettachCollision(std::string obj)
{
    std::vector<std::string> det_obj;
    det_obj.push_back(obj);
    _moveGroup->detachObject(obj);
    sleep(1.0);
    _planScene.removeCollisionObjects(det_obj);
    det_obj.clear();
    return 0;
}

int ClassicActuator::showObjectMash(std::string obj_path)
{
    ros::NodeHandle n;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = _boxPose.header.frame_id;
    collision_object.id = "object";
    shapes::Mesh* m = shapes::createMeshFromResource(obj_path);
    shape_msgs::Mesh shelf_mesh;
    shapes::ShapeMsg shelf_mesh_msg;
    // 从mesh文件指针创建mesh的msg
    shapes::constructMsgFromShape(m,shelf_mesh_msg);
    shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);

    collision_object.meshes.push_back(shelf_mesh);

    collision_object.mesh_poses.push_back(_boxPose.pose);
    collision_object.operation = collision_object.ADD;

    _collisionObjects.push_back(collision_object);
    _planScene.addCollisionObjects(_collisionObjects);
    ROS_INFO("Add an shelf into the world");
    _collisionObjects.clear();
    return 0;
}

int ClassicActuator::moveToPos(PoseStamped pose)
{
    if(_pickStopFlag)
        return -1;
    int count = 0;
    MoveItErrorCode result;
    ros::Rate loop_rate(10);

    setMoveitConfig();

    geometry_msgs::PoseStamped TPose;
    TPose.header.frame_id = pose.frame_id;
    TPose.pose.position.x = pose.pose.position.x;
    TPose.pose.position.y = pose.pose.position.y;
    TPose.pose.position.z = pose.pose.position.z;

    TPose.pose.orientation.w = pose.pose.orientation.w;
    TPose.pose.orientation.x = pose.pose.orientation.x;
    TPose.pose.orientation.y = pose.pose.orientation.y;
    TPose.pose.orientation.z = pose.pose.orientation.z;

    _moveGroup->setPoseTarget(TPose);

    ROS_INFO("moveToPos ready!!!!");

    while(count < 3 && result != MoveItErrorCode::SUCCESS){
        if(_pickStopFlag)
            return -1;
        count ++;
        ROS_ERROR("count = %d", count);
        result = _moveGroup->move();
        ROS_ERROR("moveToPos result = %d", result);
        loop_rate.sleep();
    }
    if(!result)
        return -1;
    return 0;
}

int ClassicActuator::moveToFoName(std::string posName)
{
    if(_pickStopFlag)
        return -1;
    MoveItErrorCode result;

    setMoveitConfig();

    _moveGroup->setNamedTarget(posName);
    result = _moveGroup->move();
    ROS_ERROR("move from name result = %d", result);

    return 0;
}

int ClassicActuator::quaternionToEuler(Quaternion quat,euler &pick_euler)
{
    tf::Quaternion quatt;
    quatt.setW(quat.w);
    quatt.setX(quat.x);
    quatt.setY(quat.y);
    quatt.setZ(quat.z);

    tf::Matrix3x3(quatt).getRPY(pick_euler.roll,pick_euler.pitch,pick_euler.yaw);

    return 0;
}

int ClassicActuator::eulerToQuaternion(euler euler_angle, Quaternion &quat)
{
    tf::Quaternion tq;
    tq = tf::createQuaternionFromRPY(euler_angle.roll,euler_angle.pitch,euler_angle.yaw);
    quat.w = tq.w();
    quat.x = tq.x();
    quat.y = tq.y();
    quat.z = tq.z();
    return 0;
}

int ClassicActuator::getDiraction(geometry_msgs::PoseStamped pose, std::vector<double> Roll,
                                  std::vector<double> Pitch, std::vector<double> Yaw, pick_vect& vect,
                                  std::vector<Quaternion>& quats)
{
    euler pick_euler;
    Quaternion quat;

    quat.w = pose.pose.orientation.w;
    quat.x = pose.pose.orientation.x;
    quat.y = pose.pose.orientation.y;
    quat.z = pose.pose.orientation.z;

    quaternionToEuler(quat, pick_euler);

    if((fabs(pick_euler.pitch - 1.5708) <= esp) && pick_euler.yaw == 0){
        vect.vect_x = 0;
        vect.vect_y = 0;
        vect.vect_z = -(sin(pick_euler.pitch));
    }
    else{
        vect.vect_x = cos(pick_euler.yaw);
        vect.vect_y = sin(pick_euler.yaw);
        vect.vect_z = -sin(pick_euler.pitch);
    }

    makeMoreQuat(pick_euler, Roll, Pitch, Yaw, quats);

#ifdef _COUT_
    std::cout<<"pick_euler.roll =" << pick_euler.roll <<std::endl;
    std::cout<<"pick_euler.pitch =" <<pick_euler.pitch <<std::endl;
    std::cout<<"pick_euler.yaw =" <<pick_euler.yaw <<std::endl;
    std::cout<<"quat.w =" <<pose.pose.orientation.w <<std::endl;
    std::cout<<"quat.x =" <<pose.pose.orientation.x <<std::endl;
    std::cout<<"quat.y =" <<pose.pose.orientation.y <<std::endl;
    std::cout<<"quat.z =" <<pose.pose.orientation.z <<std::endl;
    std::cout<<"vect.vect_x =" <<vect.vect_x <<std::endl;
    std::cout<<"vect.vect_y =" <<vect.vect_y <<std::endl;
    std::cout<<"vect.vect_z =" <<vect.vect_z <<std::endl;
#endif

    return 0;
}

int ClassicActuator::makeMoreQuat(euler euler_angle, std::vector<double> Roll, std::vector<double> Pitch,
                                      std::vector<double> Yaw, std::vector<Quaternion>& quats)
{
    euler remake_euler;
    Quaternion remake_quat;

    remake_euler = euler_angle;

    for(int i=0; i < Roll.size(); i++){
        for(int j=0; j < Pitch.size(); j++){
            for(int k = 0 ; k < Yaw.size(); k++){

                remake_euler.roll += Roll[i];
                remake_euler.pitch += Roll[j];
                remake_euler.yaw += Roll[k];

                eulerToQuaternion(remake_euler, remake_quat);

                quats.push_back(remake_quat);
            }
        }
    }
}

trajectory_msgs::JointTrajectory ClassicActuator::makeGripperPosture(double jiont_position)
{
    trajectory_msgs::JointTrajectory tj;
    trajectory_msgs::JointTrajectoryPoint tjp;
    tj.joint_names.push_back(m_parm.gripperConfig.joint_name);
    tjp.positions.push_back(jiont_position);
    tjp.effort.push_back(1.0);
    tjp.time_from_start = ros::Duration(1.0);
    tj.points.push_back(tjp);
    return tj;
}

int ClassicActuator::makeGrasp()
{
    pick_vect vect, pre_vect,back_vect;
    std::vector<Quaternion> quats;
    moveit_msgs::Grasp grasp_pose;
    _graspPoses.clear();

    getDiraction(m_pickPose, m_parm.pickConfig.m_Roll, m_parm.pickConfig.m_Pitch,
                 m_parm.pickConfig.m_Yaw, vect, quats);

    if(m_parm.pickConfig.pre_vect_x == 0 && m_parm.pickConfig.pre_vect_y == 0 && m_parm.pickConfig.pre_vect_z == 0){
        pre_vect = vect;
    }
    else{
        pre_vect.vect_x = m_parm.pickConfig.pre_vect_x;
        pre_vect.vect_y = m_parm.pickConfig.pre_vect_y;
        pre_vect.vect_z = m_parm.pickConfig.pre_vect_z;
    }
    if(m_parm.pickConfig.back_vect_x == 0 && m_parm.pickConfig.back_vect_y == 0 && m_parm.pickConfig.back_vect_z == 0){
        back_vect.vect_x = -(vect.vect_x);
        back_vect.vect_y = -(vect.vect_y);
        back_vect.vect_z = -(vect.vect_z);
    }
    else{
        back_vect.vect_x = m_parm.pickConfig.back_vect_x;
        back_vect.vect_y = m_parm.pickConfig.back_vect_y;
        back_vect.vect_z = m_parm.pickConfig.back_vect_z;
    }

    grasp_pose.grasp_pose.header.frame_id = m_pickPose.header.frame_id;

    //抓取前位置
    grasp_pose.pre_grasp_approach.direction.header.frame_id = m_pickPose.header.frame_id;
    grasp_pose.pre_grasp_approach.direction.vector.x = pre_vect.vect_x;
    grasp_pose.pre_grasp_approach.direction.vector.y = pre_vect.vect_y;
    grasp_pose.pre_grasp_approach.direction.vector.z = pre_vect.vect_z;
    grasp_pose.pre_grasp_approach.min_distance = m_parm.pickConfig.pre_dist_min;
    grasp_pose.pre_grasp_approach.desired_distance = m_parm.pickConfig.pre_dist_max;
    //抓取后的位置
    grasp_pose.post_grasp_retreat.direction.header.frame_id = m_pickPose.header.frame_id;
    grasp_pose.post_grasp_retreat.direction.vector.x = back_vect.vect_x;
    grasp_pose.post_grasp_retreat.direction.vector.y = back_vect.vect_y;
    grasp_pose.post_grasp_retreat.direction.vector.z = back_vect.vect_z;
    grasp_pose.post_grasp_retreat.min_distance = m_parm.pickConfig.back_dist_min;
    grasp_pose.post_grasp_retreat.desired_distance = m_parm.pickConfig.back_dist_max;

    //抓取位置
    grasp_pose.grasp_pose.pose.position.x = m_pickPose.pose.position.x;
    grasp_pose.grasp_pose.pose.position.y = m_pickPose.pose.position.y;
    grasp_pose.grasp_pose.pose.position.z = m_pickPose.pose.position.z;

    grasp_pose.allowed_touch_objects.push_back("object");
    grasp_pose.max_contact_force = 0;
    grasp_pose.pre_grasp_posture = makeGripperPosture(m_parm.gripperConfig.open_position);
    grasp_pose.grasp_posture = makeGripperPosture(m_parm.gripperConfig.close_position);

    std::stringstream ss;
    //if(quats.size() > 0){
    if(false){
        for(int i = 0; i<quats.size();i++){
            ss.clear();
            grasp_pose.grasp_pose.pose.orientation.w = quats[i].w;
            grasp_pose.grasp_pose.pose.orientation.x = quats[i].x;
            grasp_pose.grasp_pose.pose.orientation.y = quats[i].y;
            grasp_pose.grasp_pose.pose.orientation.z = quats[i].z;

            ss<<i;
            grasp_pose.id = ss.str();
            _graspPoses.push_back(grasp_pose);
        }
    }
    else{
        grasp_pose.grasp_pose.pose.orientation.w = m_pickPose.pose.orientation.w;
        grasp_pose.grasp_pose.pose.orientation.x = m_pickPose.pose.orientation.x;
        grasp_pose.grasp_pose.pose.orientation.y = m_pickPose.pose.orientation.y;
        grasp_pose.grasp_pose.pose.orientation.z = m_pickPose.pose.orientation.z;

        grasp_pose.id = "0";
        _graspPoses.push_back(grasp_pose);
    }

    return 0;
}

int ClassicActuator::pick()
{
    int count = 0;
    MoveItErrorCode result;
    ros::Rate loope(10);
    _pickStopFlag = false;
    PoseStamped pickPose;

//    moveToFoName("home");

    setMoveitConfig();

    IDebug("pick ready!!!!!");

    if(m_parm.pickConfig.direct){
        pickPose.frame_id = m_pickPose.header.frame_id;
        pickPose.pose.position.x = m_pickPose.pose.position.x;
        pickPose.pose.position.y = m_pickPose.pose.position.y;
        pickPose.pose.position.z = m_pickPose.pose.position.z;
        pickPose.pose.orientation.w = m_pickPose.pose.orientation.w;
        pickPose.pose.orientation.x = m_pickPose.pose.orientation.x;
        pickPose.pose.orientation.y = m_pickPose.pose.orientation.y;
        pickPose.pose.orientation.z = m_pickPose.pose.orientation.z;

        return moveToPos(pickPose);
    }

    makeGrasp();

    allowed.setEntry("grisper_boby_link", "obkect", true);

    IDebug("_graspPoses.size()=%d\n", _graspPoses.size());
    
    while(count < (_graspPoses.size() + 3) && result != MoveItErrorCode::SUCCESS){
         if(_pickStopFlag)
             return -1;
         ROS_ERROR("count = %d\n", count);
         result = _moveGroup->pick("object", _graspPoses);
         loope.sleep();
         count ++;
         ROS_ERROR("result = %d\n", result);
    }
    if(!result){
        return -1;
    }
    return 0;
}


int ClassicActuator::makePlace()
{
    pick_vect vect;
    std::vector<Quaternion> quats;
    moveit_msgs::PlaceLocation placeLocPos;
    _placeLocPoses.clear();

    getDiraction(m_pickPose, m_parm.placeConfig.m_Roll, m_parm.placeConfig.m_Pitch,
                 m_parm.placeConfig.m_Yaw, vect, quats);

    placeLocPos.place_pose.header.frame_id = m_placePose.header.frame_id;

    //放置进给位置
    placeLocPos.pre_place_approach.direction.header.frame_id = m_placePose.header.frame_id;
    placeLocPos.pre_place_approach.direction.vector.x = m_parm.placeConfig.pre_vect_x;
    placeLocPos.pre_place_approach.direction.vector.y = m_parm.placeConfig.pre_vect_y;
    placeLocPos.pre_place_approach.direction.vector.z = m_parm.placeConfig.pre_vect_z;
    placeLocPos.pre_place_approach.min_distance = m_parm.placeConfig.pre_dist_min;
    placeLocPos.pre_place_approach.desired_distance = m_parm.placeConfig.pre_dist_max;

    //放置回退位置
    placeLocPos.post_place_retreat.direction.header.frame_id = m_placePose.header.frame_id;
    placeLocPos.post_place_retreat.direction.vector.x = m_parm.placeConfig.back_vect_x;
    placeLocPos.post_place_retreat.direction.vector.y = m_parm.placeConfig.back_vect_y;
    placeLocPos.post_place_retreat.direction.vector.z = m_parm.placeConfig.back_vect_z;
    placeLocPos.post_place_retreat.min_distance = m_parm.placeConfig.back_dist_min;
    placeLocPos.post_place_retreat.desired_distance = m_parm.placeConfig.back_dist_max;

    //放置位置
    placeLocPos.place_pose.pose.position.x = m_placePose.pose.position.x;
    placeLocPos.place_pose.pose.position.y = m_placePose.pose.position.y;
    placeLocPos.place_pose.pose.position.z = m_placePose.pose.position.z;
    placeLocPos.allowed_touch_objects.push_back("object");
    //placeLocPos.max_contact_force = 0;
    placeLocPos.post_place_posture = makeGripperPosture(m_parm.gripperConfig.close_position);
    placeLocPos.post_place_posture = makeGripperPosture(m_parm.gripperConfig.open_position);

    std::stringstream ss;
    if(quats.size() > 0){
        for(int i = 0; i<quats.size();i++){
            ss.clear();
            placeLocPos.place_pose.pose.orientation.w = m_placePose.pose.orientation.w;
            placeLocPos.place_pose.pose.orientation.x = m_placePose.pose.orientation.x;
            placeLocPos.place_pose.pose.orientation.y = m_placePose.pose.orientation.y;
            placeLocPos.place_pose.pose.orientation.z = m_placePose.pose.orientation.z;

            ss<<i;
            placeLocPos.id = ss.str();
            _placeLocPoses.push_back(placeLocPos);
        }
    }
    else{
        placeLocPos.place_pose.pose.orientation.w = m_placePose.pose.orientation.w;
        placeLocPos.place_pose.pose.orientation.x = m_placePose.pose.orientation.x;
        placeLocPos.place_pose.pose.orientation.y = m_placePose.pose.orientation.y;
        placeLocPos.place_pose.pose.orientation.z = m_placePose.pose.orientation.z;
        placeLocPos.id = "0";
        _placeLocPoses.push_back(placeLocPos);
    }

    return 0;
}

int ClassicActuator::place()
{
    PoseStamped pose;
    int count = 0;
    int ret = -1;
    MoveItErrorCode result;
    ros::Rate loope(10);
    _pickStopFlag = false;

    IDebug("place start!!!!!");

    setMoveitConfig();

    if(m_parm.placeConfig.direct){

        result = MoveItErrorCode::SUCCESS;

        pose.frame_id = m_placePose.header.frame_id;
        pose.pose.position.x = m_placePose.pose.position.x;
        pose.pose.position.y = m_placePose.pose.position.y;
        pose.pose.position.z = m_placePose.pose.position.z;

        pose.pose.orientation.w = m_placePose.pose.orientation.w;
        pose.pose.orientation.x = m_placePose.pose.orientation.x;
        pose.pose.orientation.y = m_placePose.pose.orientation.y;
        pose.pose.orientation.z = m_placePose.pose.orientation.z;

        ret = moveToPos(pose);
    }
    else{
        ret = 0;
        makePlace();

        IDebug("place ready!!!!!");

        while(count < (_placeLocPoses.size() + 3) && result != MoveItErrorCode::SUCCESS){
            if(_pickStopFlag)
                return -1;
            ROS_ERROR("count = %d\n", count);
            result = _moveGroup->place("object",_placeLocPoses);
            loope.sleep();
            count ++;
            ROS_ERROR("result = %d\n", result);
        }
    }
    sleep(0.5);
    dettachCollision("object");
    gripperOpen(500);
    sleep(0.5);
    removeCollision();
    sleep(0.5);
   // moveToFoName("home");
    if(!result || ret != 0){
        return -1;
    }
    return 0;
}

int ClassicActuator::stopPickplace()
{
    _pickStopFlag = true;
    _moveGroup->stop();
    sleep(1.0);
    dettachCollision("object");
    sleep(2.0);
    removeCollision();
    return 0;
}

int ClassicActuator::gripperOpen(int speed)
{
	ros::ServiceClient client_gripperOpen = npick.serviceClient<hsr_gripper_driver::open_srv>("gripper_open");
    hsr_gripper_driver::open_srv gripperOpen_srv;
	gripperOpen_srv.request.speed = speed;
    if(client_gripperOpen.call(gripperOpen_srv)){
        IDebug("Open the Gripper sucessful!!!");
	}
	else{
        IErrorPrint("Open the Gripper failed!!!");
	}	
}

 H_EXPORT_PLUGIN(ClassicActuator, "ClassicActuator", "1.0")

