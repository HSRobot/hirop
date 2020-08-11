---
力控阻抗算法 	
/**
     * @brief 解析私有参数  
     * @return  
     */
    virtual int parseConfig(YAML::Node& ) = 0;

    /**
     * @brief setInputForce 输入力矩参数
     * @return
     */
    virtual void setInputForceBias() = 0;

    /**
     * @brief setInputRobotPose 输入机器人的笛卡尔坐标
     * @return
     */
    virtual void setInputRobotCartPose() = 0;

    /**
     * @brief compute 算法运算
     * @return
     */
    virtual int compute() = 0;

    /**
     * @brief getResult 获取力控运算的输出结果
     * @return
     */
    virtual int getResult(std::vector<double> &) = 0;


    /**
     * @brief swithMode  切换模式 
     * @param type 0 关节 1 笛卡尔模式
     * @return
     */
    virtual int swithMode(int type) = 0;

    /**
     * @brief getName  获取算法插件的名称
     * @param name
     * @return
     */
    virtual int getName(std::string &name) = 0;

    /**
     * @brief setDirection 设置力控的方向模式
     * @param name
     * @return
     */
    virtual int setDirection(std::string &name) = 0;

    /**
     * @brief printInfo 输出打印信息
     * @return
     */
    virtual int printInfo() = 0;

