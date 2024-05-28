#include <elastoplastic_lugre_controller/elastoplastic_lugre_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(phri::control::CartImpedanceLuGreController, controller_interface::ControllerBase)


namespace phri
{
  namespace control
  {

  CartImpedanceLuGreController::~CartImpedanceLuGreController()
  {

  }

  bool CartImpedanceLuGreController::init(hardware_interface::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {    
    m_root_nh = root_nh;
    m_controller_nh = controller_nh;
    m_hw = hw;

    // m_controller_nh has a private queue where the incoming messages are stored
    m_controller_nh.setCallbackQueue(&m_queue);


    // reading frames of the chain
    if (!m_controller_nh.getParam("base_frame",m_base_frame)) // base of the robot
    {
      ROS_ERROR("%s/base_frame not defined", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("tool_frame",m_tool_frame))  // tool of the robot
    {
      ROS_ERROR("%s/tool_frame not defined", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("sensor_frame",m_sensor_frame))  // frame of the sensor
    {
      ROS_ERROR("%s/sensor_frame not defined", m_controller_nh.getNamespace().c_str());
      return false;
    }

    // if true,  the base is the reference of impedance equations (x=x_base, y=y_base, z=z_base)
    // if false, the tool is the reference of impedance equations (x=x_tool, y=y_tool, z=z_tool)
    if (!m_controller_nh.getParam("base_is_reference", m_base_is_reference))
    {
      ROS_INFO("Using a base reference Cartesian impedance as default");
      m_base_is_reference=true;
    }

    if (!m_base_is_reference)
    {
      ROS_ERROR("tool reference is not implemented yet");
      return false;
    }

    // reading the information of the robot structure
    urdf::Model urdf_model;
    if (!urdf_model.initParam("/robot_description"))
    {
      ROS_ERROR("Urdf robot_description '%s' does not exist",(m_controller_nh.getNamespace()+"/robot_description").c_str());
      return false;
    }

    // read gravity vector
    std::vector<double> gravity_vec;
    Eigen::Vector3d gravity;
    if (!m_controller_nh.getParam("gravity",gravity_vec))  // tool of the robot
    {
      gravity << 0, 0, -9.806;
    }
    else
    {
      if (gravity_vec.size()!=3)
      {
        ROS_ERROR("%s/gravity is not a vector of three elements", m_controller_nh.getNamespace().c_str());
        throw std::invalid_argument("gravity is not a vector of three elements");
      }
      gravity(0)=gravity_vec.at(0);
      gravity(1)=gravity_vec.at(1);
      gravity(2)=gravity_vec.at(2);
    }


    // create chains
    m_chain_bt = rosdyn::createChain(urdf_model,m_base_frame,m_tool_frame,gravity);  // base <- tool
    m_chain_bs = rosdyn::createChain(urdf_model,m_base_frame,m_sensor_frame,gravity); // base <- sensor

    // limit cartesian velocity and acceleration
    m_cartesian_limits_ok = true;
    if (!m_controller_nh.getParam("cart_velocity_limit", m_cart_velocity_limit ))
    {
      ROS_WARN("%s/cart_velocity_limit does not exist. Unable to use scaling", m_controller_nh.getNamespace().c_str());
      m_cartesian_limits_ok = false;
    }
    if (!m_controller_nh.getParam("cart_acceleration_limit", m_cart_acceleration_limit ))
    {
      ROS_WARN("%s/cart_acceleration_limit does not exist. Unable to use scaling", m_controller_nh.getNamespace().c_str());
      m_cartesian_limits_ok = false;
    }
    if (!m_controller_nh.getParam("cart_force_limit", m_cart_force_limit ))
    {
      ROS_WARN("%s/cart_force_limit does not exist. Unable to use scaling", m_controller_nh.getNamespace().c_str());
      m_cartesian_limits_ok = false;
    }

    // actuated joints
    if (!controller_nh.getParam("controlled_joint",m_joint_names))
    {
      ROS_INFO("/controlled_joint not specified, using all");
      m_joint_names=m_hw->getNames();
    }

    m_nAx=m_joint_names.size();
    m_chain_bs->setInputJointsName(m_joint_names);
    m_chain_bt->setInputJointsName(m_joint_names);

    m_joint_handles.resize(m_nAx);
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
      m_joint_handles.at(iAx)=m_hw->getHandle(m_joint_names.at(iAx));


    m_target.resize(m_nAx);  // joint target position
    m_Dtarget.resize(m_nAx); // joint target velocity
    m_x.resize(m_nAx);  // algorithm output position
    m_Dx.resize(m_nAx);   // algorithm output velocity
    m_DDx.resize(m_nAx);   // algorithm output acceleration
    m_q.resize(m_nAx);
    m_Dq.resize(m_nAx);
    m_DDq.resize(m_nAx);
    m_velocity_limits.resize(m_nAx);
    m_effort_limits.resize(m_nAx);
    m_wrench_of_tool_in_base_with_deadband.resize(6);

    m_idle_Jinv.resize(6);  // inverse of the desired inertia (idle)
    m_idle_damping.resize(6);  // (absolute) damping coefficient (idle)
    m_trj_Jinv.resize(6);  // inverse of the desired inertia (trj)
    m_trj_damping.resize(6);  // (absolute) damping coefficient (trj)
    m_k.resize(6);  // spring


    m_velocity_limits.resize(m_nAx);
    m_acceleration_limits.resize(m_nAx);
    m_upper_limits.resize(m_nAx);
    m_lower_limits.resize(m_nAx);

    m_acc_deadband.resize(6);


    // read limits from urdf and/or moveit
    for (unsigned int iAx=0; iAx<m_nAx; iAx++)
    {
      m_upper_limits(iAx) = urdf_model.getJoint(m_joint_names.at(iAx))->limits->upper;
      m_lower_limits(iAx) = urdf_model.getJoint(m_joint_names.at(iAx))->limits->lower;


      if ((m_upper_limits(iAx)==0) && (m_lower_limits(iAx)==0))
      {
        m_upper_limits(iAx)=std::numeric_limits<double>::infinity();
        m_lower_limits(iAx)=-std::numeric_limits<double>::infinity();
        ROS_INFO("upper and lower limits are both equal to 0, set +/- infinity");
      }

      bool has_velocity_limits;
      if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/has_velocity_limits",has_velocity_limits))
        has_velocity_limits=false;
      bool has_acceleration_limits;
      if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/has_acceleration_limits",has_acceleration_limits))
        has_acceleration_limits=false;

      m_velocity_limits(iAx)= urdf_model.getJoint(m_joint_names.at(iAx))->limits->velocity;

      if (has_velocity_limits)
      {
        double vel;
        if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_velocity",vel))
        {
          ROS_ERROR_STREAM("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_velocity is not defined");
          return false;
        }
        if (vel<m_velocity_limits(iAx))
          m_velocity_limits(iAx)=vel;
      }

      if (has_acceleration_limits)
      {
        double acc;
        if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_acceleration",acc))
        {
          ROS_ERROR_STREAM("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_acceleration is not defined");
          return false;
        }
        m_acceleration_limits(iAx)=acc;
      }
      else
        m_acceleration_limits(iAx)=10*m_velocity_limits(iAx);
    }

    // Impedance Parameters
    std::vector<double> inertia, damping, stiffness, wrench_deadband;
    // Inertia
    if (!m_controller_nh.getParam("inertia", inertia))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/inertia does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (inertia.size()!=6)
    {
      ROS_ERROR("inertia should be have six values");
      return false;
    }

    /*
    // Stiffness
    if (!m_controller_nh.getParam("stiffness", stiffness))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/stiffness does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (stiffness.size()!=6)
    {
      ROS_ERROR("%s/stiffness should be have six values", m_controller_nh.getNamespace().c_str());
      return false;
    }

    // Damping (or Damping Ratio)
    // if damping_ratio is defined, using relative damping
    if (m_controller_nh.hasParam("damping_ratio"))
    {
      std::vector<double> damping_ratio;
      if (!m_controller_nh.getParam("damping_ratio_idle", damping_ratio))
      {
        ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/damping_ratio is not a vector of doubles");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return false;
      }

      if (damping_ratio.size()!=6)
      {
        ROS_ERROR("damping should be have six values");
        return false;
      }

      damping.resize(6,0);
      for (unsigned int iAx=0;iAx<6;iAx++)
      {
        if (stiffness.at(iAx)<=0)
        {
          ROS_ERROR("damping ratio can be specified only for positive stiffness values (stiffness of Joint %s is not positive)",m_joint_names.at(iAx).c_str());
          return false;
        }
        damping.at(iAx)=2*damping_ratio.at(iAx)*std::sqrt(stiffness.at(iAx)*inertia.at(iAx));
      }
    }
    else if (!m_controller_nh.getParam("damping_idle", damping)) // if damping_ratio is not defined, using absolute damping
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/damping does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    */

    if (!m_controller_nh.getParam("damping", damping)) // if damping_ratio is not defined, using absolute damping
    {
          ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/damping does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
    }

    if (damping.size()!=6)
    {
      ROS_ERROR("%s/damping should have six values", m_controller_nh.getNamespace().c_str());
      return false;
    }


    // Wrench Deadband
    if (!m_controller_nh.getParam("wrench_deadband", wrench_deadband))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/wrench_deadband does not exist, set to zero");
      wrench_deadband.resize(6,0);
    }

    if (wrench_deadband.size()!=6)
    {
      ROS_ERROR("%s/wrench_deadband should be have six values", m_controller_nh.getNamespace().c_str());
      return false;
    }


    // LuGre Parameters
    if (!m_controller_nh.getParam("sigma0", m_idle_sigma0 ))
    {
      ROS_ERROR("%s/sigma0 does not exist", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (!m_controller_nh.getParam("sigma1", m_idle_sigma1 ))
    {
      ROS_ERROR("%s/sigma1 does not exist", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (!m_controller_nh.getParam("c0", m_idle_c0 ))
    {
      ROS_ERROR("%s/c0 does not exist", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (!m_controller_nh.getParam("z_ss", m_idle_z_ss ))
    {
      ROS_WARN("%s/z_ss does not exist. Using c0/sigma0", m_controller_nh.getNamespace().c_str());
      m_idle_z_ss = m_idle_c0/m_idle_sigma0;
    }

    if (!m_controller_nh.getParam("z_ba", m_idle_z_ba ))
    {
      ROS_WARN("%s/z_ba does not exist. Using 0.9*z_ss", m_controller_nh.getNamespace().c_str());
      m_idle_z_ba = 0.9*m_idle_z_ss;
    }

//    if (!m_controller_nh.getParam("mu_k", m_mu_k ))
//    {
//      ROS_ERROR("%s/mu_k does not exist, set 0.78", m_controller_nh.getNamespace().c_str());
//      m_mu_k=0.78;
//    }


    for (unsigned int iAx=0;iAx<6;iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        ROS_ERROR("inertia value of Joint %s is not positive",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_idle_Jinv(iAx)=1.0/inertia.at(iAx);
      }

      if (damping.at(iAx)<=0)
      {
        ROS_ERROR("damping value of Joint %s is not positive",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_idle_damping(iAx)=damping.at(iAx);
      }

/*
      if (stiffness.at(iAx)<0)
      {
        ROS_ERROR("stiffness value of Joint %s is negative",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_k(iAx)=stiffness.at(iAx);
      }
*/

      if (wrench_deadband.at(iAx)<0)
      {
        ROS_INFO("wrench_deadband %d is negative, set zero",iAx);
        m_wrench_deadband(iAx)=0.0;
      }
      else
        m_wrench_deadband(iAx)=wrench_deadband.at(iAx);
    }
    // End Idle Parameters

    // Trajectory following parameters
    std::vector<double> inertia_trj, damping_trj;
    // Inertia
    if (!m_controller_nh.getParam("inertia_trj", inertia_trj))
    {
      ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/inertia_trj does not exist. Using "+m_controller_nh.getNamespace()+"/intertia instead");
      inertia_trj = inertia;
    }
    if (inertia_trj.size()!=6)
    {
      ROS_ERROR("inertia should be have six values");
      return false;
    }

    if (!m_controller_nh.getParam("damping_trj", damping_trj)) // if damping_ratio is not defined, using absolute damping
    {
      ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/damping_trj does not exist. Using "+m_controller_nh.getNamespace()+"/damping instead");
      damping_trj = damping;
    }
    if (damping_trj.size()!=6)
    {
      ROS_ERROR("%s/damping should have six values", m_controller_nh.getNamespace().c_str());
      return false;
    }

    for (unsigned int iAx=0;iAx<6;iAx++)
    {
      if (inertia_trj.at(iAx)<=0)
      {
        ROS_ERROR("inertia value of Joint %s is not positive",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_trj_Jinv(iAx)=1.0/inertia_trj.at(iAx);
      }

      if (damping_trj.at(iAx)<=0)
      {
        ROS_ERROR("damping value of Joint %s is not positive",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_trj_damping(iAx)=damping_trj.at(iAx);
      }
    }

    // LuGre Parameters
//    if (!m_controller_nh.getParam("sigma0_trj", m_trj_sigma0 ))
//    {
//      ROS_WARN("%s/sigma0_trj does not exist. Using %s/sigma0.", m_controller_nh.getNamespace().c_str(), m_controller_nh.getNamespace().c_str());
//      m_trj_sigma0 = m_idle_sigma0;
//    }

//    if (!m_controller_nh.getParam("sigma1_trj", m_trj_sigma1 ))
//    {
//      ROS_WARN("%s/sigma1_trj does not exist. Using %s/sigma1.", m_controller_nh.getNamespace().c_str(), m_controller_nh.getNamespace().c_str());
//      m_trj_sigma1 = m_idle_sigma1;
//    }

//    if (!m_controller_nh.getParam("c0_trj", m_trj_c0 ))
//    {
//      ROS_WARN("%s/c0_trj does not exist. Using %s/c0.", m_controller_nh.getNamespace().c_str(),m_controller_nh.getNamespace().c_str());
//      m_trj_c0 = m_idle_c0;
//    }

//    if (!m_controller_nh.getParam("z_ss_trj", m_trj_z_ss ))
//    {
//      ROS_WARN("%s/z_ss does not exist. Using c0/sigma0", m_controller_nh.getNamespace().c_str());
//      m_trj_z_ss = m_trj_c0/m_trj_sigma0;
//    }

//    if (!m_controller_nh.getParam("z_ba_trj", m_trj_z_ba ))
//    {
//      ROS_WARN("%s/z_ba_trj does not exist. Using 0.9*z_ss", m_controller_nh.getNamespace().c_str());
//      m_trj_z_ba = 0.9*m_trj_z_ss;
//    }

    // Initial values
    m_sigma0 = m_idle_sigma0;
    m_sigma1 = m_idle_sigma1;
    m_c0 = m_idle_c0;
    m_z_ss = m_idle_z_ss;
    m_z_ba = m_idle_z_ba;
    m_Jinv = m_idle_Jinv;
    m_damping = m_idle_damping;

    // Acceleration deadband
    std::vector<double> acc_deadband;
    if (!m_controller_nh.getParam("acceleration_deadband",acc_deadband)){
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/acceleration_deadband doesn't exist, set to zero");
      acc_deadband.resize(6,0);
    }
    for(unsigned int i=0; i < 6; i++){
      m_acc_deadband(i) = acc_deadband.at(i);
    }

    ROS_INFO("Controller '%s' controls the following joints:",m_controller_nh.getNamespace().c_str());
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      ROS_INFO(" - %s",m_joint_names.at(iAx).c_str());
      ROS_INFO("position limits = [%f, %f]",m_lower_limits(iAx),m_upper_limits(iAx));
      ROS_INFO("velocity limits = [%f, %f]",-m_velocity_limits(iAx),m_velocity_limits(iAx));
      ROS_INFO("acceleration limits = [%f, %f]",-m_acceleration_limits(iAx),m_acceleration_limits(iAx));
    }

    // Reset time constant
//    if (!m_controller_nh.getParam("Tp",m_Tp)){
//      ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/Tp does not exist. Reset time constant set to default value: 0.5");
//      m_Tp = 0.5;
//    }
    // Controller for angular accelerations
    if (!m_controller_nh.getParam("kp_acceleration",m_Kp_ang_acc)){
      ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/kp_acceleration does not exist, set to default: 1");
      m_Kp_ang_acc = 1;
    }
    if (!m_controller_nh.getParam("ks_acceleration",m_Ks_ang_acc)){
      ROS_INFO_STREAM(m_controller_nh.getNamespace()+"/ks_acceleration does not exist, set to default: 1");
      m_Ks_ang_acc = 1;
    }


    // subscribe topics
    std::string joint_target = "joint_target_topic";
    std::string external_wrench = "external_wrench";
    if (!m_controller_nh.getParam("joint_target_topic", joint_target))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/joint_target_topic does not exist. Default value 'joint_target_topic' superimposed");
      joint_target = "joint_target_topic";
    }

    if (!m_controller_nh.getParam("external_wrench_topic", external_wrench ))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/external_wrench does not exist. Default value 'external_wrench' superimposed");
      external_wrench = "external_wrench";
    }

    if (!m_controller_nh.getParam("interpolation_to_trj",T_to_trj))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/interpolation_to_trj does not exist. Set to default: 1");
      T_to_trj = 1;
    }
    if(T_to_trj <= 0)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/interpolation_to_trj value is less or equal to zero. Set to 0.1");
      T_to_trj = 0.1;
    }

    if (!m_controller_nh.getParam("interpolation_to_idle",T_to_idle))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/interpolation_to_idle does not exist. Set to default: 1");
      T_to_idle = 1;
    }
    if(T_to_idle <= 0)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/interpolation_to_idle value is less or equal to zero. Set to 0.1");
      T_to_idle = 0.1;
    }

    if (!m_controller_nh.getParam("trj_ratio_limit",m_trj_ratio_limit))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/trj_ratio_limit does not exist. Set to default: 1");
      m_trj_ratio_limit = 1;
    }
    if (m_trj_ratio_limit > 1 || m_trj_ratio_limit < 0) {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/trj_ratio_limit greater than 1 or less than 0. Set to default: 1");
      m_trj_ratio_limit = 1;
    }

    if(!m_controller_nh.getParam("kw", m_kw))
    {
      m_kw = 1.0;
      ROS_WARN_STREAM(m_controller_nh.getNamespace() << "/kw not set. Using default: " << m_kw);
    }
    if(!m_controller_nh.getParam("reset_window_size", m_reset_window_size))
    {
      m_reset_window_size = 1000; // in milliseconds, integer
      ROS_WARN_STREAM(m_controller_nh.getNamespace() << "/reset_window_size does not exists. Set to default: " << m_reset_window_size);
    }
    if(!m_controller_nh.getParam("reset_threshold", m_reset_threshold))
    {
      m_reset_threshold = 1.0;
      ROS_WARN_STREAM(m_controller_nh.getNamespace() << "/reset_threshold does not exists. Set to default: " << m_reset_threshold);
    }


    //Subscribers
    m_target_sub=std::make_shared<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>>(m_controller_nh,joint_target,1);
    m_target_sub->setAdvancedCallback(boost::bind(&phri::control::CartImpedanceLuGreController::setTargetCallback,this,_1));

    m_wrench_sub=std::make_shared<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>>(m_controller_nh,external_wrench,1);
    m_wrench_sub->setAdvancedCallback(boost::bind(&phri::control::CartImpedanceLuGreController::setWrenchCallback,this,_1));

    m_exec_ratio_sub=std::make_shared<ros_helper::SubscriptionNotifier<std_msgs::Float64>>(m_controller_nh,"/execution_ratio",1);
    m_exec_ratio_sub->setAdvancedCallback(boost::bind(&phri::control::CartImpedanceLuGreController::getRatioCallback,this,_1));

    // Publishers
    m_pub_z = m_controller_nh.advertise<std_msgs::Float64MultiArray>("z", 1);
    m_pub_w = m_controller_nh.advertise<std_msgs::Float64MultiArray>("w", 1);
    m_pub_cerr = m_controller_nh.advertise<std_msgs::Float64MultiArray>("cart_err",1);
    m_pub_F_fr = m_controller_nh.advertise<geometry_msgs::WrenchStamped>("Fr_in_base",1);
    m_pub_x =  m_controller_nh.advertise<sensor_msgs::JointState>("x",1);
    m_pub_wrench_in_base = m_controller_nh.advertise<geometry_msgs::WrenchStamped>("wrench_in_base",1);
    m_pub_pose_of_t_in_b = m_controller_nh.advertise<geometry_msgs::PoseStamped>("pose_of_t_in_b",1);
    m_pub_target_of_t_in_b = m_controller_nh.advertise<geometry_msgs::PoseStamped>("target_of_t_in_b",1);

    ROS_DEBUG("Subscribing to %s",joint_target.c_str());
    ROS_DEBUG("Subscribing to %s",external_wrench.c_str());

    ROS_INFO("Controller '%s' well initialized",m_controller_nh.getNamespace().c_str());

    return true;
  }



  void CartImpedanceLuGreController::starting(const ros::Time& time)
  {
    m_target.setZero();
    m_Dtarget.setZero();
    m_x.setZero();
    m_Dx.setZero();
    m_DDx.setZero();
    m_q.setZero();
    m_Dq.setZero();
    m_DDq.setZero();
    m_alpha.setZero();
    m_wrench_of_tool_in_base_with_deadband.setZero();

    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      m_x(iAx)=m_joint_handles.at(iAx).getPosition(); // read actual position
      m_Dx(iAx)=m_joint_handles.at(iAx).getVelocity(); // read actual position
      m_joint_handles.at(iAx).setCommand(m_x(iAx),m_Dx(iAx),0.0);//  send position and velocity command to lower level
    }
    m_target=m_x;
    m_Dtarget=m_Dx;

    // Deformation
    m_cart_vel_of_t_in_b.setZero();
    m_cart_pos_of_t_in_b.setZero();

    m_z.setZero();  // lugre state
    m_Dz.setZero();

    m_w.setZero();
    m_Dw.setZero();

    m_r = 0;
    m_Dr = 0;

    trj_status = Idle;

    m_queue.callAvailable(); // check for new messages

    ROS_INFO("Controller '%s' well started at time %f",m_controller_nh.getNamespace().c_str(),time.toSec());
    m_is_configured = (m_target_ok && m_effort_ok);
    if (m_is_configured)
      ROS_INFO("configured");
  }

  void CartImpedanceLuGreController::stopping(const ros::Time& time)
  {
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      m_x(iAx)=m_joint_handles.at(iAx).getPosition(); // read actual position
      m_Dx(iAx)=m_joint_handles.at(iAx).getVelocity(); // read actual position
      m_joint_handles.at(iAx).setCommand(m_x(iAx),m_Dx(iAx),0.0); //  send position and velocity command to lower level
    }
    ROS_INFO("[ %s ] Stopping controller at time %f", m_controller_nh.getNamespace().c_str(),time.toSec());
  }

  void CartImpedanceLuGreController::update(const ros::Time& time, const ros::Duration& period)
  {
    try
    {
      m_queue.callAvailable();
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Something wrong in the callback: %s",e.what());
    }
    m_is_configured = (m_target_ok && m_effort_ok); // true if there are messages from the subscribed topics

    // is robot following a trj?
    if ((trj_status==Idle) && (m_execution_ratio < 1.0))
    {
      trj_status=TransitionToTrjFollowing;
      t_start_switch=ros::Time::now();
      ROS_INFO("New state: Transition to Trajectory Following");
    }
    else if ((trj_status==TransitionToTrjFollowing) && (ros::Time::now()-t_start_switch).toSec()>T_to_trj)
    {
      trj_status=TrjFollowing;
      ROS_INFO("New state: Trajectory Following");
    }
    else if ((trj_status==TrjFollowing) && m_execution_ratio >= m_trj_ratio_limit)
    {
      trj_status = TransitionToIdle;
      t_start_switch=ros::Time::now();
      ROS_INFO("New state: Transition to Idle");
    }
    else if ((trj_status==TransitionToIdle) && (ros::Time::now()-t_start_switch).toSec()>T_to_idle)
    {
      trj_status = Idle;
      ROS_INFO("New state: Idle");
    }

    // Transformation matrix  base <- target pose of the tool
    Eigen::Affine3d T_base_targetpose = m_chain_bt->getTransformation(m_target);
    // Jacobian of the target in base reference
    Eigen::Matrix6Xd J_base_target  = m_chain_bt->getJacobian(m_target);
    // velocity of the target in base reference  v=J*Dq
    Eigen::Vector6d cart_vel_target_in_b  = J_base_target*m_Dtarget;

    // Transformation matrix  base <- tool
    Eigen::Affine3d T_b_t = m_chain_bt->getTransformation(m_x);
    // Jacobian of the tool in base reference
    Eigen::Matrix6Xd J_of_t_in_b  = m_chain_bt->getJacobian(m_x);
    // velocity of the tool in base reference  v=J*Dq
    Eigen::Vector6d cart_vel_of_t_in_b  = J_of_t_in_b*m_Dx;

    // acceleration of the tool in base reference a= D(J*Dq)=D(J)*Dq+J*DDq
    Eigen::Vector6d cart_acc_of_t_in_b;
    cart_acc_of_t_in_b.setZero();
    // non linear part of the acceleration (centrifugal, Coriolis)
    Eigen::Vector6d cart_acc_nl_of_t_in_b  = m_chain_bt->getDTwistNonLinearPartTool(m_x,m_Dx); // D(J)*Dq

    // Cartesian error between tool pose and target tool pose
    Eigen::VectorXd cartesian_error_actual_target_in_b;
    //rosdyn::getFrameDistance(T_base_targetpose, T_b_t , cartesian_error_actual_target_in_b);
    rosdyn::getFrameDistance(T_b_t, T_base_targetpose, cartesian_error_actual_target_in_b);
    Eigen::VectorXd cartesian_error_velocity_target_in_b;
    cartesian_error_velocity_target_in_b = cart_vel_of_t_in_b - cart_vel_target_in_b;

    Eigen::Vector4d alpha_r;
    alpha_r << m_z, m_r;

    m_z_norm = m_z.head(3).norm();
    m_vel_norm = cartesian_error_velocity_target_in_b.head(3).norm();

    auto alpha = [this](const double z) -> double
    {
      if (std::abs(z) < m_z_ba)
      {
        return 0.0;
      }
      else if (std::abs(z) >= m_z_ss)
      {
        return 1.0;
      }
      else
      {
        return 0.5*std::sin(M_PI*((z-(m_z_ba+m_z_ss)/2)/(m_z_ss-m_z_ba)))+0.5;
      }
    };

    auto dalpha = [this](const double z, const double zp) -> double
    {
      if (std::abs(z) < m_z_ba)
      {
        return 0.0;
      }
      else if (std::abs(z) >= m_z_ss)
      {
        return 0.0;
      }
      else
      {
        return 0.5*std::cos(M_PI*((z-(m_z_ba+m_z_ss)/2)/(m_z_ss-m_z_ba)))*M_PI/(m_z_ss-m_z_ba)*std::abs(zp);
      }
    };

    if (m_base_is_reference)
    {
//      double lambda=std::pow(m_mu_k,2.0)*m_vel_norm;
//      double lambda = m_mu_k * m_vel_norm;
//      for (int i=0;i<m_z.size();i++)
//      {
//        if (std::abs(m_z(i)) < m_z_ba)
//        {
//          m_alpha(i) = 0.0;
//        }
//        else if (std::abs(m_z(i)) >= m_z_ss)
//        {
//          m_alpha(i) = 1.0;
//        }
//        else
//        {
//          m_alpha(i) = 0.5*std::sin(M_PI*((m_z(i)-(m_z_ba+m_z_ss)/2)/(m_z_ss-m_z_ba)))+0.5;
//        }
//        m_scale(i) = (1.0-m_alpha(i));
////        double lambda_1 = lambda/m_c0;
////        m_c0_v(i) = m_sigma0*lambda_1*m_alpha(i)/(m_mu_k*m_mu_k);
//        m_c0_v(i) = m_alpha(i) * m_sigma0 * m_vel_norm / m_c0;
//        m_Dw(i) = m_alpha(i) * 20 * (m_z(i) - m_w(i)) - m_scale(i) * 50 * m_w(i);
//      }

//      m_alpha = Eigen::Vector3d({alpha(m_z_norm), alpha(m_z_norm), alpha(m_z_norm)});
      m_alpha = Eigen::Vector3d::Constant(alpha(alpha_r.norm()));
      m_c0_v =  m_alpha * m_sigma0 * m_vel_norm / m_c0;
      m_Dr = dalpha(m_z.norm(), m_Dz.norm());
      m_Dz = cartesian_error_velocity_target_in_b.head(3) - m_c0_v.cwiseProduct(m_z);
//      m_Dw = 50 * Eigen::Vector3d({alpha((m_z).norm()), alpha((m_z).norm()), alpha((m_z).norm())}).cwiseProduct(m_z - m_w);
      m_Dw = m_kw * m_alpha.cwiseProduct(m_z - m_w);
      m_F_frc =  m_sigma0*(m_z - m_w) + m_sigma1*m_Dz + m_damping.head(3).cwiseProduct(cartesian_error_velocity_target_in_b.head(3));

      geometry_msgs::WrenchStamped Fr_in_base_msg;
      std_msgs::Float64MultiArray z_msg;
      std_msgs::Float64MultiArray Dz_msg;
      std_msgs::Float64MultiArray w_msg;
      std_msgs::Float64MultiArray cerr_msg;

      for(int i=0; i < 6; i++)
        cerr_msg.data.push_back(cartesian_error_actual_target_in_b(i));
      m_pub_cerr.publish(cerr_msg);

      // Publish controller response
      Fr_in_base_msg.header.stamp=ros::Time::now();
      Fr_in_base_msg.header.frame_id=m_base_frame;
      Fr_in_base_msg.wrench.force.x=m_F_frc(0);
      Fr_in_base_msg.wrench.force.y=m_F_frc(1);
      Fr_in_base_msg.wrench.force.z=m_F_frc(2);
      m_pub_F_fr.publish(Fr_in_base_msg);

      // (Deformation) Cartesian acceleration
      cart_acc_of_t_in_b.head(3) = m_Jinv.head(3).cwiseProduct(-m_F_frc+m_wrench_of_tool_in_base_with_deadband.head(3));
      cart_acc_of_t_in_b.tail(3) = -(m_Kp_ang_acc*cartesian_error_actual_target_in_b.tail(3)+m_Ks_ang_acc*cartesian_error_velocity_target_in_b.tail(3));

      for (int i=0; i < 6; i++) {
        if (std::abs(cart_acc_of_t_in_b(i)) < std::abs(m_acc_deadband(i))){
          cart_acc_of_t_in_b(i) = 0.0;
        }
      }

      m_Dz_norm = m_Dz.norm();
      m_z = m_Dz*period.toSec() + m_z;
      m_w = m_Dw*period.toSec() + m_w;
      m_r = m_Dr*period.toSec() + m_r;

      for (int i=0;i<m_z.size();i++)
      {
//        if (std::abs(m_z(i)) > m_z_ss)
//          m_z(i) = m_z(i)/std::abs(m_z(i)) * m_z_ss;
        z_msg.data.push_back(m_z(i));
        w_msg.data.push_back(m_w(i));
      }



      z_msg.data.push_back(m_z.norm());
      m_pub_z.publish(z_msg);
      m_pub_w.publish(w_msg);

      //Reset z
      if(m_alpha.maxCoeff() > 0)
      {
        const size_t window_reset_size = (size_t) std::ceil(m_reset_window_size/(period.toSec()*1e3));
        m_reset_window.emplace_back(m_wrench_of_tool_in_base_with_deadband.head(3).transpose()*cartesian_error_velocity_target_in_b);
        if(m_reset_window.size() > window_reset_size)
        {
          m_reset_window.pop_front();
        }
        const double reset_value = std::accumulate(m_reset_window.begin(), m_reset_window.end(), 0.0,
                                             [&period](const double d, const double x) -> double
                                                      {
                                                        return d + x*period.toSec();
                                                      }
                              );
        ROS_INFO("last value inserted: %f\nreset_value: %f", m_reset_window.back(), reset_value);

        if(m_reset_window.size() >= window_reset_size &&
           reset_value < m_reset_threshold) // Valore a caso
        {
          m_z = Eigen::Vector3d::Zero();
          m_w = Eigen::Vector3d::Zero();
          m_r = 0.0;
          m_reset_window.clear();
        }
      }
      else
      {
        if(!m_reset_window.empty())
        {
          m_reset_window.clear();
        }
      }

    }
    else  //tool is reference
    {
      // TODO
    }


    //  m_err_norm = cart_err;
    m_cart_acc_of_t_in_b = cart_acc_of_t_in_b;

//    // Compute svd decomposition of jacobian (to compute joint acceleration "inverting" the Jacobian)
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_of_t_in_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
//    //Singularities
//    if (svd.singularValues()(svd.cols()-1)==0)
//      ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
//    else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
//      ROS_WARN_THROTTLE(1,"SINGULARITY POINT");


////    m_pub_acc
//    std_msgs::Float64MultiArray acc_msg;
//    acc_msg.data.push_back(cart_acc_of_t_in_b(0));
//    acc_msg.data.push_back(cart_acc_of_t_in_b(1));
//    acc_msg.data.push_back(cart_acc_of_t_in_b(2));
//    acc_msg.data.push_back(cart_acc_nl_of_t_in_b(0));
//    acc_msg.data.push_back(cart_acc_nl_of_t_in_b(1));
//    acc_msg.data.push_back(cart_acc_nl_of_t_in_b(2));
//    m_pub_acc.publish(acc_msg);


   /*
    *
    * // (deformation) cartesian acceleration =  LUGRE
    * // (global) cartesian acceleration = (target) cartesian acceleration + (deformation) cartesian acceleration
    * // IK   (global) cartesian acceleration = DJ(x)*Dx+J(x)*DDx  ==> DDx
    * // INT  DDx integrator -> x,Dx
    *
    * ==================================================================================================================================
    * m_cart_pos_of_t_in_b  += m_cart_vel_of_t_in_b  * period.toSec() + m_cart_acc_of_t_in_b*std::pow(period.toSec(),2.0)*0.5;
    *
    * m_cart_vel_of_t_in_b += m_cart_acc_of_t_in_b * period.toSec();
    *
    * //m_global_cart_pos_of_t_in_b = m_cart_pos_of_t_in_b + target_cart_pos_of_t_in_b;
    * T_target_t.translation()=m_cart_pos_of_t_in_b;
    * T_b_t = T_b_target * T_target_t;
    *
    * //m_global_art_vel_of_t_in_b = m_cart_vel_of_t_in_b + cart_vel_target_in_b;
    *
    * m_chain_bt->computeLocalIk(m_x,T_b_t,m_x);
    *
    * J_of_t_in_b  = m_chain_bt->getJacobian(m_x);
    * m_Dx=
    * =============================================================================================================================
    *
    *
    *  // POSSIBILITA` 2
    *  // (deformation) cartesian acceleration =  LUGRE
    *  // INT (deformation) cartesian acceleration ==> (deformation) velocity and position
    *  // (global) cartesian position = (target) cartesian position + (deformation) cartesian position
    *  // (global) cartesian velocity = (target) cartesian velocity + (deformation) cartesian velocity
    *  // IK  global velocity and position -> x, Dx
    *
    *
    *
    *
    *
    *
    *
    *
    *
    *
    *
    *
    *
    *  // x=target+q
    *  // Dx=Dtarget+Dq
    *  // DDx=DDtarget+DDq
    *  // (global) cartesian acceleration = (target) cartesian acceleration + ==>(deformation) cartesian acceleration<==
    *
    *
    *
    *  //  (target) cartesian acceleration = DJ(target)*Dtarget+J(target)*DDtarget
    *  //  (deformation) cartesian acceleration = (global) cartesian acceleration-(target) cartesian acceleration
    */

    /*
     *
     *                                                         +-------------+
     * T_base_targetpose                        +---+          |             |
     * cart_vel_target_in_b  +----------------->+ + +--------->+    IK()     |
     *                                          +-+-+          |             |
     *                                            ^            +-------------+
     *                         cart_vel_of_t_in_b |
     *                         cart_pos_of_t_in_b |     +-----+
     *                                            |     |  1  |
     *                                            +-----+  -  +<-----+ elastoplastic acceleration
     *                                                  |  s2 |
     *                                                  +-----+
     *
     *
     *
     */

    m_cart_pos_of_t_in_b  += m_cart_vel_of_t_in_b  * period.toSec() + m_cart_acc_of_t_in_b*std::pow(period.toSec(),2.0)*0.5; // IGNORE ROTATIONS
    m_cart_vel_of_t_in_b  += m_cart_acc_of_t_in_b * period.toSec();

    Eigen::Affine3d T_base_tool_next;
    Eigen::Vector6d V_base_tool_next;
    T_base_tool_next = Eigen::Translation3d(m_cart_pos_of_t_in_b.head(3)) * T_base_targetpose;
    V_base_tool_next = m_cart_vel_of_t_in_b + cart_vel_target_in_b;
    m_chain_bt->computeLocalIk(m_x,T_base_tool_next,m_x); // new joint position
    J_of_t_in_b = m_chain_bt->getJacobian(m_x);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_of_t_in_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Singularities
    if (svd.singularValues()(svd.cols()-1)==0)
      ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
    else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
      ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
    m_Dx = svd.solve(V_base_tool_next); // new joint velocity

    // Scaling due to joint velocity limits
    double scaling_vel = 1.0;
    for(size_t idx = 0; idx < m_nAx; idx++)
    {
      scaling_vel = std::max(ratio_acc,std::abs(m_Dx(idx))/m_velocity_limits(idx));
    }
    if(scaling_vel > 1)
    {
      m_Dx = svd.solve(scaling_vel * V_base_tool_next);
    }


    // QUESTO SALTA!
    // cartesian acceleration = D(J)*Dq+J*(DDq) -> DDq=J\(cartesian_acceleration-D(J)*Dq)
    //m_DDq = svd.solve(cart_acc_of_t_in_b-cart_acc_nl_of_t_in_b);

//    // saturate acceleration to compute distance
//    Eigen::VectorXd saturated_acc=m_DDq;
//    double ratio_acc=1;
//    for (unsigned int idx=0; idx<m_nAx; idx++)
//      ratio_acc=std::max(ratio_acc,std::abs(m_DDq(idx))/m_acceleration_limits(idx));
//    saturated_acc/=ratio_acc;

//    // check joint limit feasibility, break if needed
//    for (unsigned int idx=0; idx<m_nAx; idx++)
//    {
//      //Computing breaking distance
//      // velocity(t_break)=|current_velocity|-max_acc*t_break=0 -> t_break=|current_velocity|/max_acc
//      // distance(t_break)=|current_velocity|*t_break-0.5*max_t_break^2=0.5*max_t_break^2
//      double t_break=std::abs(m_Dx(idx))/m_acceleration_limits(idx); // breaking time
//      double breaking_distance=0.5*m_acceleration_limits(idx)*std::pow(t_break,2.0);

//      if (m_x(idx) > (m_upper_limits(idx)-breaking_distance))
//      {
//        if (m_Dx(idx)>0)
//        {
//          ROS_WARN_THROTTLE(2,"Breaking, maximum limit approaching on joint %s",m_joint_names.at(idx).c_str());
//          saturated_acc(idx)=-m_acceleration_limits(idx);
//        }
//      }
//      if (m_x(idx) < (m_lower_limits(idx) + breaking_distance))
//      {
//        if (m_Dx(idx) < 0)
//        {
//          ROS_WARN_THROTTLE(2,"Breaking, minimum limit approaching on joint %s",m_joint_names.at(idx).c_str());
//          saturated_acc(idx)=m_acceleration_limits(idx);
//        }
//      }
//    }
//    m_DDq=saturated_acc;

///*
//    // integrate acceleration
//    m_x  += m_Dx  * period.toSec() + m_DDx*std::pow(period.toSec(),2.0)*0.5;
//    m_Dx += m_DDx * period.toSec();
//*/
//    m_q  += m_Dq  * period.toSec() + m_DDq*std::pow(period.toSec(),2.0)*0.5;
//    m_Dq += m_DDq * period.toSec();

//    m_x = m_target + m_q;
//    m_Dx = m_Dtarget + m_Dq;
    // m_DDx=m_DDtarget+m_DDq
    // cart_acc=J*m_DDx+DJ*m_Dx

    // saturate position and velocity
    for (unsigned int idx=0;idx<m_nAx;idx++)
    {
      m_x(idx)=std::max(m_lower_limits(idx),std::min(m_upper_limits(idx),m_x(idx)));
      m_Dx(idx)=std::max(-m_velocity_limits(idx),std::min(m_velocity_limits(idx),m_Dx(idx)));
    }

    sensor_msgs::JointState x_msg;
    x_msg.header.stamp=ros::Time::now();
    x_msg.name=m_joint_names;
    x_msg.position.resize(m_x.size());
    x_msg.velocity.resize(m_x.size());
    for (int iax=0; iax<m_x.size();iax++)
    {
      x_msg.position.at(iax) = m_x(iax);
      x_msg.velocity.at(iax) = m_Dx(iax);
    }
    m_pub_x.publish(x_msg);

    geometry_msgs::PoseStamped pose_t_in_b;
    tf::poseEigenToMsg(T_b_t,pose_t_in_b.pose);
    pose_t_in_b.header.frame_id=m_base_frame;
    pose_t_in_b.header.stamp=ros::Time::now();
    m_pub_pose_of_t_in_b.publish(pose_t_in_b);

    geometry_msgs::PoseStamped target_t_in_b;
    tf::poseEigenToMsg(T_base_targetpose,target_t_in_b.pose);
    target_t_in_b.header.frame_id=m_base_frame;
    target_t_in_b.header.stamp=pose_t_in_b.header.stamp;
    m_pub_target_of_t_in_b.publish(target_t_in_b);

    // send position and velocity command to lower level
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
      m_joint_handles.at(iAx).setCommand(m_x(iAx),m_Dx(iAx),0.0);
  }

  // callback for the new target position messages
  void CartImpedanceLuGreController::setTargetCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    try
    {
      sensor_msgs::JointState tmp_msg=*msg;
      // resort joints in correct order
      if (!name_sorting::permutationName(m_joint_names,tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort))
      {
        ROS_ERROR("joints not found");
        m_target_ok=false;
        return;
      }
      if (!m_target_ok)
        ROS_INFO("First target message received");


      m_target_ok=true;
      for (unsigned int iAx=0;iAx<m_nAx;iAx++)
      {
        m_target(iAx)=tmp_msg.position.at(iAx);
        m_Dtarget(iAx)=tmp_msg.velocity.at(iAx);
      }
    }
    catch(...)
    {
      ROS_ERROR("Something wrong in target callback");
      m_target_ok=false;
    }
  }

  // callback for the new wrench topic
  void CartImpedanceLuGreController::setWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    // check if sensor frame is correct
    if (msg->header.frame_id.compare(m_sensor_frame)) // compare return 0 if the strings are equal
    {
      ROS_WARN("sensor frame is %s, it should be %s",msg->header.frame_id.c_str(),m_sensor_frame.c_str());
      return;
    }

    try
    {

      Eigen::Vector6d wrench_of_sensor_in_sensor; // wrench of sensor in sensor frame
      wrench_of_sensor_in_sensor(0) = msg->wrench.force.x;
      wrench_of_sensor_in_sensor(1) = msg->wrench.force.y;
      wrench_of_sensor_in_sensor(2) = msg->wrench.force.z;
      wrench_of_sensor_in_sensor(3) = msg->wrench.torque.x;
      wrench_of_sensor_in_sensor(4) = msg->wrench.torque.y;
      wrench_of_sensor_in_sensor(5) = msg->wrench.torque.z;

      Eigen::Affine3d T_base_tool=m_chain_bt->getTransformation(m_x);
      Eigen::Affine3d T_base_sensor=m_chain_bs->getTransformation(m_x);
      Eigen::Affine3d T_tool_sensor= T_base_tool.inverse()*T_base_sensor;

      // wrench of tool in tool
      Eigen::Vector6d wrench_of_tool_in_tool = rosdyn::spatialDualTranformation (wrench_of_sensor_in_sensor , T_tool_sensor         );

      // wrench of tool in base
      Eigen::Vector6d wrench_of_tool_in_base = rosdyn::spatialRotation          (wrench_of_tool_in_tool     , T_base_tool.linear()  );

      for (unsigned int idx=0;idx<6;idx++)
      {
        if ( (wrench_of_tool_in_base(idx)>m_wrench_deadband(idx)))
          m_wrench_of_tool_in_base_with_deadband(idx)=wrench_of_tool_in_base(idx)-m_wrench_deadband(idx);
        else if ( (wrench_of_tool_in_base(idx)<-m_wrench_deadband(idx)))
          m_wrench_of_tool_in_base_with_deadband(idx)=wrench_of_tool_in_base(idx)+m_wrench_deadband(idx);
        else
          m_wrench_of_tool_in_base_with_deadband(idx)=0;
      }

      geometry_msgs::WrenchStamped wrench_in_base_msg;
      wrench_in_base_msg.header.stamp=msg->header.stamp;
      wrench_in_base_msg.header.frame_id=m_base_frame;
      // force
      wrench_in_base_msg.wrench.force.x=m_wrench_of_tool_in_base_with_deadband(0);
      wrench_in_base_msg.wrench.force.y=m_wrench_of_tool_in_base_with_deadband(1);
      wrench_in_base_msg.wrench.force.z=m_wrench_of_tool_in_base_with_deadband(2);
      // torque
      wrench_in_base_msg.wrench.torque.x=m_wrench_of_tool_in_base_with_deadband(3);
      wrench_in_base_msg.wrench.torque.y=m_wrench_of_tool_in_base_with_deadband(4);
      wrench_in_base_msg.wrench.torque.z=m_wrench_of_tool_in_base_with_deadband(5);


      m_pub_wrench_in_base.publish(wrench_in_base_msg);

      if (!m_effort_ok)
        ROS_INFO("First wrench message received");
      m_effort_ok=true;
    }
    catch(...)
    {
      ROS_ERROR("Something wrong in wrench callback");
      m_effort_ok=false;
    }

  }

  void CartImpedanceLuGreController::getRatioCallback(const std_msgs::Float64::ConstPtr& msg){
    m_execution_ratio = msg->data;
    ROS_INFO_ONCE("Execution Ratio Received");
    if (m_execution_ratio > 1){
      ROS_WARN("Execution ratio greater than 1. Using 1");
      m_execution_ratio = 1;
    }
    else if (m_execution_ratio < 0) {
      ROS_WARN("Execution ratio is negative. Using 0");
      m_execution_ratio = 0;
    }
  }

  }  // namespace control
}  // namespace phri
