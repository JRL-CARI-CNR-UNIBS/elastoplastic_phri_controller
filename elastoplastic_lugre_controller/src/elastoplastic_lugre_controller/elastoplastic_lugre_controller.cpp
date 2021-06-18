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

    m_Jinv.resize(6);  // inverse of the desired inertia
    m_damping.resize(6);  // (absolute) damping coefficient
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

    // read impedance parameters
    std::vector<double> inertia, damping, stiffness, wrench_deadband;
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

    // if damping_ratio is defined, using relative damping
    if (m_controller_nh.hasParam("damping_ratio"))
    {
      std::vector<double> damping_ratio;
      if (!m_controller_nh.getParam("damping_ratio", damping_ratio))
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
    else if (!m_controller_nh.getParam("damping", damping)) // if damping_ratio is not defined, using absolute damping
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/damping does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (damping.size()!=6)
    {
      ROS_ERROR("%s/damping should be have six values", m_controller_nh.getNamespace().c_str());
      return false;
    }


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


    // lugre parameters
    if (!m_controller_nh.getParam("sigma0", m_sigma0 ))
    {
      ROS_ERROR("%s/sigma0 does not exist", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (!m_controller_nh.getParam("sigma1", m_sigma1 ))
    {
      ROS_ERROR("%s/sigma1 does not exist", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (!m_controller_nh.getParam("c0", m_c0 ))
    {
      ROS_ERROR("%s/c0 does not exist", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (!m_controller_nh.getParam("z_ss", m_z_ss ))
    {
      ROS_ERROR("%s/z_ss does not exist", m_controller_nh.getNamespace().c_str());
      return false;
    }

    if (!m_controller_nh.getParam("z_ba", m_z_ba ))
    {
      ROS_ERROR("%s/z_ba does not exist", m_controller_nh.getNamespace().c_str());
      return false;
    }


    if (!m_controller_nh.getParam("mu_k", m_mu_k ))
    {
      ROS_ERROR("%s/mu_k does not exist, set 0.78", m_controller_nh.getNamespace().c_str());
      m_mu_k=0.78;
    }


    for (unsigned int iAx=0;iAx<6;iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        ROS_ERROR("inertia value of Joint %s is not positive",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_Jinv(iAx)=1.0/inertia.at(iAx);
      }

      if (damping.at(iAx)<=0)
      {
        ROS_ERROR("damping value of Joint %s is not positive",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_damping(iAx)=damping.at(iAx);
      }

      if (stiffness.at(iAx)<0)
      {
        ROS_ERROR("stiffness value of Joint %s is negative",m_joint_names.at(iAx).c_str());
        return false;
      }
      else
      {
        m_k(iAx)=stiffness.at(iAx);
      }

      if (wrench_deadband.at(iAx)<0)
      {
        ROS_INFO("wrench_deadband %d is negative, set zero",iAx);
        m_wrench_deadband(iAx)=0.0;
      }
      else
        m_wrench_deadband(iAx)=wrench_deadband.at(iAx);
    }

    // Acceleration deadband
    std::vector<double> acc_deadband;
    if (!m_controller_nh.getParam("acceleration_deadband",acc_deadband)){
      ROS_WARN("Acceleration deadband doesn't exists, set to zero");
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

    //Reset time constant
    if (!m_controller_nh.getParam("Tp",m_Tp)){
      ROS_INFO("Reset time constant to default value: 0.5");
    }

    if (!m_controller_nh.getParam("kp_acceleration",m_Kp_ang_acc)){
      ROS_INFO("Missing kp for angular acceleration, set to default = 1");
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

    m_pub_z = m_controller_nh.advertise<std_msgs::Float64MultiArray>("z",1); // Publisher: z
    m_pub_cerr = m_controller_nh.advertise<std_msgs::Float64MultiArray>("cart_err",1); // Publisher: cartesian error

    m_target_sub=std::make_shared<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>>(m_controller_nh,joint_target,1);
    m_target_sub->setAdvancedCallback(boost::bind(&phri::control::CartImpedanceLuGreController::setTargetCallback,this,_1));

    m_wrench_sub=std::make_shared<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>>(m_controller_nh,external_wrench,1);
    m_wrench_sub->setAdvancedCallback(boost::bind(&phri::control::CartImpedanceLuGreController::setWrenchCallback,this,_1));

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

    m_z.setZero();  // lugre state
    m_Dz.setZero();

    m_queue.callAvailable(); // check for new messages

    m_alpha_prec.setZero();

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

    // Transformation matrix  base <- target pose of the tool
    Eigen::Affine3d T_base_targetpose = m_chain_bt->getTransformation(m_target);

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
    rosdyn::getFrameDistance(T_base_targetpose, T_b_t , cartesian_error_actual_target_in_b);

    m_z_norm = m_z.head(3).norm();
    m_vel_norm = cart_vel_of_t_in_b.head(3).norm();

    if (m_base_is_reference)
    {
      double lambda=std::pow(m_mu_k,2.0)*cart_vel_of_t_in_b.head(3).norm();
      for (int i=0;i<m_z.size();i++)
      {
        if (std::abs(m_z_norm) < m_z_ba)
        {
          m_alpha(i) = 0.0;
        }
        else if (std::abs(m_z_norm) >= m_z_ss)
        {
          m_alpha(i) = 1.0;
        }
        else
        {
          m_alpha(i) = 0.5*std::sin(M_PI*((m_z_norm-(m_z_ba+m_z_ss)/2)/(m_z_ss-m_z_ba)))+0.5;
        }
        m_scale(i) = (1.0-m_alpha(i));
        double lambda_1 = lambda/m_c0;
        m_c0_v(i) = m_sigma0*lambda_1*m_alpha(i)/(m_mu_k*m_mu_k);
      }

      m_Dz = cart_vel_of_t_in_b.head(3) - m_c0_v.cwiseProduct(m_z);
      m_F_frc =  m_sigma0*m_z.cwiseProduct(m_scale) + m_sigma1*m_Dz + m_damping.head(3).cwiseProduct(cart_vel_of_t_in_b.head(3));


      // Cartesian acceleration
      cart_acc_of_t_in_b.head(3) = m_Jinv.head(3).cwiseProduct(-m_F_frc+m_wrench_of_tool_in_base_with_deadband.head(3));
      cart_acc_of_t_in_b.tail(3) = m_Kp_ang_acc*cartesian_error_actual_target_in_b.tail(3);

      for (int i=0; i < 6; i++) {
        if (std::abs(cart_acc_of_t_in_b(i)) < std::abs(m_acc_deadband(i))){
          cart_acc_of_t_in_b(i) = 0.0;
        }
      }

      //ROS_INFO_STREAM_THROTTLE(1,"Errore Cartesiano:\n " << cartesian_error_actual_target_in_b);
      //ROS_INFO_STREAM_THROTTLE(1,"Accelerazioni:\n " << cart_acc_of_t_in_b);
      //ROS_INFO_STREAM_THROTTLE(1,"Deadband accelerazioni:\n " << m_acc_deadband.transpose());

      m_Dz_norm = m_Dz.norm();
      m_z = m_Dz*period.toSec() + m_z;

      for (int i=0;i<m_z.size();i++)
      {
        if (std::abs(m_z(i)) > m_z_ss)
          m_z(i) = m_z(i)/std::abs(m_z(i)) * m_z_ss;
      }     

      //Reset z
      double norm_Dx = m_Dx.norm();
      //double D_norm_Dx = (norm_Dx - m_old_Dx_norm)/period.toSec(); // Accelerazione
      double D_norm_Dx = (m_DDx.dot(m_Dx))/norm_Dx; //Derivata esatta di norm_Dx
      double zp1,zp2; // Previsione di ||z|| al tempo Tp e 2Tp
      zp1 = norm_Dx*m_Tp + 0.5*D_norm_Dx*std::pow(m_Tp,2);
      zp2 = norm_Dx*2*m_Tp + 0.5*D_norm_Dx*std::pow(2*m_Tp,2);
      if (std::abs(zp1) < m_z_ba && std::abs(zp2) < m_z_ba && m_alpha(1) > 0.999){
        ROS_WARN("Reset of z");
        m_z.setZero();
      }

      std_msgs::Float64MultiArray z_msg;
      std_msgs::Float64MultiArray cerr_msg;

      for(int i=0; i< 3; i++)
        z_msg.data.push_back(m_z(i));
      z_msg.data.push_back(m_z.norm());
      m_pub_z.publish(z_msg);

      for(int i=0; i < 6; i++)
        cerr_msg.data.push_back(cartesian_error_actual_target_in_b(i));
      m_pub_cerr.publish(cerr_msg);


      // Vecchio reset: Fisso
/*    double al = std::max(m_alpha(0),std::max(m_alpha(1),m_alpha(2)));
      if (std::abs(m_Dz_norm-m_vel_norm) < 0.001)
        if (al == 1.0)
          for (int i=0;i<3;i++)
            m_z(i) = 0;//(1.0-std::exp(-std::abs(m_vel_norm)/0.001));//m_Dz(i)*period.toSec() + m_z(i)*(std::abs(m_vel_norm)/0.5);// std::exp(-std::abs(m_vel_norm)/0.1);
*/
    }
    else  //tool is reference
    {
      // TODO
    }


    //  m_err_norm = cart_err;
    m_cart_acc_of_t_in_b = cart_acc_of_t_in_b;

    // Compute svd decomposition of jacobian (to compute joint acceleration "inverting" the Jacobian)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_of_t_in_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Singularities
    if (svd.singularValues()(svd.cols()-1)==0)
      ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
    else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
      ROS_WARN_THROTTLE(1,"SINGULARITY POINT");

    // cartesian acceleration = D(J)*Dq+J*DDq -> DDq=J\(cartesian_acceleration-D(J)*Dq)
    m_DDq = svd.solve(cart_acc_of_t_in_b-cart_acc_nl_of_t_in_b);

    // saturate acceleration to compute distance
    Eigen::VectorXd saturated_acc=m_DDq;
    double ratio_acc=1;
    for (unsigned int idx=0; idx<m_nAx; idx++)
      ratio_acc=std::max(ratio_acc,std::abs(m_DDq(idx))/m_acceleration_limits(idx));
    saturated_acc/=ratio_acc;

    // check joint limit feasibility, break if needed
    for (unsigned int idx=0; idx<m_nAx; idx++)
    {
      //Computing breaking distance
      // velocity(t_break)=|current_velocity|-max_acc*t_break=0 -> t_break=|current_velocity|/max_acc
      // distance(t_break)=|current_velocity|*t_break-0.5*max_t_break^2=0.5*max_t_break^2
      double t_break=std::abs(m_Dx(idx))/m_acceleration_limits(idx); // breaking time
      double breaking_distance=0.5*m_acceleration_limits(idx)*std::pow(t_break,2.0);

      if (m_x(idx) > (m_upper_limits(idx)-breaking_distance))
      {
        if (m_Dx(idx)>0)
        {
          ROS_WARN_THROTTLE(2,"Breaking, maximum limit approaching on joint %s",m_joint_names.at(idx).c_str());
          saturated_acc(idx)=-m_acceleration_limits(idx);
        }
      }
      if (m_x(idx) < (m_lower_limits(idx) + breaking_distance))
      {
        if (m_Dx(idx) < 0)
        {
          ROS_WARN_THROTTLE(2,"Breaking, minimum limit approaching on joint %s",m_joint_names.at(idx).c_str());
          saturated_acc(idx)=m_acceleration_limits(idx);
        }
      }
    }
    m_DDq=saturated_acc;

/*
    // integrate acceleration
    m_x  += m_Dx  * period.toSec() + m_DDx*std::pow(period.toSec(),2.0)*0.5;
    m_Dx += m_DDx * period.toSec();
*/
    m_q  += m_Dq  * period.toSec() + m_DDq*std::pow(period.toSec(),2.0)*0.5;
    m_Dq += m_DDq * period.toSec();

    m_x = m_target + m_q;
    m_Dx = m_Dtarget + m_Dq;

    // saturate position and velocity
    for (unsigned int idx=0;idx<m_nAx;idx++)
    {
      m_x(idx)=std::max(m_lower_limits(idx),std::min(m_upper_limits(idx),m_x(idx)));
      m_Dx(idx)=std::max(-m_velocity_limits(idx),std::min(m_velocity_limits(idx),m_Dx(idx)));
    }

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

  }  // namespace control
}  // namespace phri
