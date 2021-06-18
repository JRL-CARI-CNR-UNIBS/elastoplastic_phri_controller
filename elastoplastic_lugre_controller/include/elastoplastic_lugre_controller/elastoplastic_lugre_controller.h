#ifndef phri_cart_impedanceLuGre__
#define phri_cart_impedanceLuGre__

# include <ros/ros.h>
# include <ros/callback_queue.h>
# include <cnr_hardware_interface/posveleff_command_interface.h>
# include <controller_interface/controller.h>
# include <subscription_notifier/subscription_notifier.h>
# include <sensor_msgs/JointState.h>
# include <geometry_msgs/WrenchStamped.h>
# include <std_msgs/Float64MultiArray.h>
# include <rosdyn_core/primitives.h>
# include <name_sorting/name_sorting.h>
# include <ros/timer.h>

namespace phri
{
  namespace control
  {

    class CartImpedanceLuGreController : public controller_interface::Controller<hardware_interface::PosVelEffJointInterface>
    {
    public:
      // initialize controller (called once when the controller is created)
      bool init(hardware_interface::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
      // update controller (called every period)
      void update(const ros::Time& time, const ros::Duration& period);
      // starting controller (called every time the controller is started)
      void starting(const ros::Time& time);
      // stopping controller (called every time the controller is stopped)
      void stopping(const ros::Time& time);


    protected:

      hardware_interface::PosVelEffJointInterface* m_hw;
      std::vector<hardware_interface::PosVelEffJointHandle> m_joint_handles;
      ros::NodeHandle m_root_nh;
      ros::NodeHandle m_controller_nh;
      ros::CallbackQueue m_queue;


      unsigned int m_nAx;
      std::vector< std::string > m_joint_names;
      std::string m_base_frame;
      std::string m_tool_frame;
      std::string m_sensor_frame;
      bool m_base_is_reference;
      rosdyn::ChainPtr m_chain_bt;
      rosdyn::ChainPtr m_chain_bs;
      Eigen::Vector3d grav;


      Eigen::VectorXd m_target;
      Eigen::VectorXd m_Dtarget;

      // delta relative
      Eigen::VectorXd m_q;
      Eigen::VectorXd m_Dq;
      Eigen::VectorXd m_DDq;

      // coordinate giunti assolute
      Eigen::VectorXd m_x;
      Eigen::VectorXd m_Dx;
      Eigen::VectorXd m_DDx;

      Eigen::VectorXd m_velocity_limits;
      Eigen::VectorXd m_acceleration_limits;
      Eigen::VectorXd m_effort_limits;
      Eigen::VectorXd m_upper_limits;
      Eigen::VectorXd m_lower_limits;
      double m_cart_velocity_limit;
      double m_cart_acceleration_limit;
      double m_cart_force_limit;

      Eigen::Vector6d m_Jinv;
      Eigen::Vector6d m_damping;
      Eigen::Vector6d m_k;
      Eigen::Vector6d m_wrench_deadband;

      Eigen::Vector6d m_wrench_of_tool_in_base_with_deadband;
      Eigen::Vector6d m_old_wrench_of_tool_in_base_with_deadband;
      double m_wrench_dev;
      Eigen::Affine3d m_T_tool_sensor;
      Eigen::Affine3d m_T_base_tool;

      double m_vel_norm;
      double m_Dz_norm;

      double m_mu_k;
      double m_wrench_norm;
      double m_vel_old;
      double m_acc_norm;
      double m_force_norm;
      Eigen::Vector6d m_err_norm;


      bool m_is_configured;
      bool m_target_ok;
      bool m_effort_ok;
      bool m_cartesian_limits_ok;


      std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> m_target_sub;
      std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>> m_wrench_sub;

      //LuGre parameters
      Eigen::Vector3d m_z;
      double m_z_norm;
      Eigen::Vector3d m_Dz;
      double m_sigma0;
      Eigen::Vector3d m_scale;
      double m_sigma1;
      double m_c0;
      double m_z_ba;
      double m_z_ss;
      Eigen::Vector3d m_F_frc;
      Eigen::Vector3d m_alpha;
      Eigen::Vector3d m_c0_v;

      Eigen::Vector6d m_acc_LuGre;
      Eigen::Vector6d m_vel_LuGre;
      Eigen::Vector6d m_pos_LuGre;
      Eigen::Vector3d m_old_F_angle;
      Eigen::Vector3d m_angle_count;
      Eigen::Vector3d m_old_angle_count;
      Eigen::Vector3d m_Dangle_count;
      Eigen::Vector3d m_real_F_angle;

      double m_old_Dx_norm = 0.0;
      double m_Tp = 0.5;
      Eigen::Vector6d m_acc_deadband;
      double m_Kp_ang_acc = 1;

      Eigen::Vector6d m_cart_vel_of_t_in_b;

      bool m_bool_act_reset = false;
      Eigen::Vector6d m_cart_acc_of_t_in_b;

      ros::Publisher m_pub_z;
      ros::Publisher m_pub_cerr;
      ros::Publisher m_pub_Dx;
      ros::Publisher m_pub_F_fr;



      Eigen::Vector3d m_alpha_prec;
      Eigen::Vector3d m_max_Dz;

      void setTargetCallback(const sensor_msgs::JointStateConstPtr& msg);
      void setWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg);

      ~CartImpedanceLuGreController();

    };


  }
}


#endif
