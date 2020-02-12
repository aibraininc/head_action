#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <cmath>

#include <actionlib/server/action_server.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <urdf/model.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Float32MultiArray.h>


typedef actionlib::ActionServer<control_msgs::PointHeadAction> PHAS;
typedef PHAS::GoalHandle GoalHandle;


class HeadAction
{
private:
  std::vector< std::string> joint_names_;
  ros::ServiceClient cli_query_traj_;

public:
  

  ros::NodeHandle nh_;
  PHAS action_server_;
  std::string action_name_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  urdf::Model urdf_model_;

  ros::Publisher pub_controller_command_;

  HeadAction(const ros::NodeHandle &node):
  	action_name_("absolute_point_head_action"), nh_(node),
  	action_server_(nh_, "absolute_point_head_action",
                       boost::bind(&HeadAction::goalCB, this, _1),
                       boost::bind(&HeadAction::cancelCB, this, _1), false)
  {
    if (tree_.getNrOfJoints() == 0)
    {
      std::string robot_desc_string;
      std::string robot_desc_key;
      if(nh_.searchParam("robot_description", robot_desc_key))
      {
        ROS_INFO_STREAM("Load description from: " << robot_desc_key);
        nh_.param(robot_desc_key, robot_desc_string, std::string());
      }

      
      ROS_DEBUG("Reading tree from robot_description...");
      if (!kdl_parser::treeFromString(robot_desc_string, tree_))
      {
         ROS_ERROR("Failed to construct kdl tree");
         exit(-1);
      }
      if (!urdf_model_.initString(robot_desc_string))
      {
        ROS_ERROR("Failed to parse urdf string for urdf::Model.");
        exit(-2);
      }
    }
  	action_server_.start();

    // Connects to the controller
    pub_controller_command_ =
      nh_.advertise<trajectory_msgs::JointTrajectory>("command", 2);

    cli_query_traj_ =
        nh_.serviceClient<control_msgs::QueryTrajectoryState>("query_state");
  }

  void goalCB(GoalHandle gh)
  {

    boost::scoped_ptr<KDL::ChainFkSolverPos> pose_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    double success_angle_threshold_ = 0.03;

    const geometry_msgs::PointStamped &target = gh.getGoal()->target;
    tf::Point target_in_root_;
    target_in_root_[0] = target.point.x;
    target_in_root_[1] = target.point.y;
    target_in_root_[2] = target.point.z;

    // get chain_
    bool success = tree_.getChain("base_link", "camera_color_optical_frame", chain_);
    const unsigned int joints = chain_.getNrOfJoints();
    KDL::JntArray jnt_pos(joints), jnt_eff(joints);
    KDL::Jacobian jacobian(joints);

    pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
    joint_names_.resize(chain_.getNrOfJoints());    
    std::string pointing_frame_ = gh.getGoal()->pointing_frame;

    control_msgs::QueryTrajectoryState traj_state;
    traj_state.request.time = ros::Time::now() + ros::Duration(0.01);
    if (!cli_query_traj_.call(traj_state))
    {
      ROS_ERROR("Service call to query controller trajectory failed.");
      gh.setRejected();
      return;
    }
    if (traj_state.response.name.size() != joints)
    {
      ROS_ERROR("Number of joints mismatch: urdf chain vs. trajectory controller state.");
      gh.setRejected();
      return;
    }
    std::vector<urdf::JointLimits> limits_(joints);
    // Get initial joint positions and joint limits.
    for (unsigned int i = 0; i < joints; ++i)
    {
      joint_names_[i] = traj_state.response.name[i];
      limits_[i] = *(urdf_model_.joints_[joint_names_[i].c_str()]->limits);
      ROS_DEBUG("Joint %d %s: %f, limits: %f %f", i, traj_state.response.name[i].c_str(), traj_state.response.position[i], limits_[i].lower, limits_[i].upper);
      jnt_pos(i) = 0;
    }

    ROS_INFO("He11o 2");

    int count = 0;
    int limit_flips = 0;
    float correction_angle = 2*M_PI;
    float correction_delta = 2*M_PI;
    const int MAX_ITERATIONS = 15;


    while( ros::ok() &&
           fabs(correction_delta) > 0.001 &&
           count < MAX_ITERATIONS) //limit the iterations
    {
      //get the pose and jacobian for the current joint positions
      KDL::Frame pose;
      pose_solver_->JntToCart(jnt_pos, pose);
      jac_solver_->JntToJac(jnt_pos, jacobian);

      tf::Transform frame_in_root;
      tf::poseKDLToTF(pose, frame_in_root);

      tf::Vector3 pointing_axis_;
      tf::vector3MsgToTF(gh.getGoal()->pointing_axis, pointing_axis_);


      tf::Vector3 axis_in_frame = pointing_axis_.normalized();
      tf::Vector3 target_from_frame = (target_in_root_ - frame_in_root.getOrigin()).normalized();
      tf::Vector3 current_in_frame = frame_in_root.getBasis().inverse()*target_from_frame;
      float prev_correction = correction_angle;
      correction_angle = current_in_frame.angle(axis_in_frame);
      correction_delta = correction_angle - prev_correction;

      ROS_INFO("At step %d, joint poses are %.4f and %.4f, angle error is %f radians", count, jnt_pos(0), jnt_pos(1), correction_angle);
      double goal_error_ = correction_angle; //expected error after this iteration
      if ( correction_angle < 0.5*success_angle_threshold_ )
      {
        ROS_DEBUG_STREAM("Accepting solution as estimated error is: " << correction_angle*180.0/M_PI << "degrees and stopping condition is half of " << success_angle_threshold_*180.0/M_PI);
        break;
      }
      tf::Vector3 correction_axis = frame_in_root.getBasis()*(axis_in_frame.cross(current_in_frame).normalized());
      tf::Transform correction_tf(tf::Quaternion(correction_axis, 0.5*correction_angle), tf::Vector3(0,0,0));
      KDL::Frame correction_kdl;
      tf::transformTFToKDL(correction_tf, correction_kdl);

      // We apply a "wrench" proportional to the desired correction
      KDL::Frame identity_kdl;
      KDL::Twist twist = diff(correction_kdl, identity_kdl);
      KDL::Wrench wrench_desi;
      for (unsigned int i=0; i<6; ++i)
        wrench_desi(i) = -1.0*twist(i);

      // Converts the "wrench" into "joint corrections" with a jacbobian-transpose
      for (unsigned int i = 0; i < joints; ++i)
      {
        jnt_eff(i) = 0;
        for (unsigned int j=0; j<6; ++j)
          jnt_eff(i) += (jacobian(j,i) * wrench_desi(j));
        jnt_pos(i) += jnt_eff(i);
      }
      count++;

      if (limit_flips > 1)
      {
        ROS_ERROR("Goal is out of joint limits, trying to point there anyway... \n");
        break;
      }
    }
    std::vector<double> q_goal(joints);
    for(unsigned int i = 0; i < joints; i++)
      q_goal[i] = jnt_pos(i);

    ROS_INFO_STREAM("q_goal " <<q_goal[0] <<", "<<q_goal[1]);

    //re-compute desired pointing axis from desired joint positions 
    //(to take the case in which joint limits have been enforced into account)
    KDL::Frame pose;
    pose_solver_->JntToCart(jnt_pos, pose);

    tf::Transform frame_in_root;
    tf::poseKDLToTF(pose, frame_in_root);
    tf::Vector3 target_from_frame = target_in_root_ - frame_in_root.getOrigin();
    target_from_frame.normalize();
    tf::Vector3 desired_pointing_axis_in_frame_ = frame_in_root.getBasis().inverse()*target_from_frame;
    ROS_INFO_STREAM("desired_pointing_axis_in_frame_ " <<desired_pointing_axis_in_frame_[0] <<", "<<desired_pointing_axis_in_frame_[1]<<", "<< desired_pointing_axis_in_frame_[2]);

    // Computes the duration of the movement.
    ros::Duration min_duration(0.01);
    if (gh.getGoal()->min_duration > min_duration)
        min_duration = gh.getGoal()->min_duration;

    // Determines if we need to increase the duration of the movement in order to enforce a maximum velocity.
    if (gh.getGoal()->max_velocity > 0)
    {
      // compute the largest required rotation among all the joints
      double largest_rotation = 0;
      for(unsigned int i = 0; i < joints; i++)
      {
        double required_rotation = fabs(q_goal[i] - traj_state.response.position[i]);
        if ( required_rotation > largest_rotation )
          largest_rotation = required_rotation;
      }

      ros::Duration limit_from_velocity(largest_rotation / gh.getGoal()->max_velocity);
      if (limit_from_velocity > min_duration)
        min_duration = limit_from_velocity;
    }

    // Computes the command to send to the trajectory controller.
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = traj_state.request.time;

    traj.joint_names.push_back(traj_state.response.name[0]);
    traj.joint_names.push_back(traj_state.response.name[1]);

    traj.points.resize(1);
    traj.points[0].positions = q_goal;
    traj.points[0].velocities.push_back(0);
    traj.points[0].velocities.push_back(0);
    traj.points[0].time_from_start = ros::Duration(min_duration);
    pub_controller_command_.publish(traj);
    ROS_INFO_STREAM("goalCB");
  }
  void cancelCB(GoalHandle gh)
  {
    ROS_INFO_STREAM("cancelCB");
  }  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "headAction");
  ros::NodeHandle node;
  HeadAction ch(node);
  ros::spin();
  return 0;	
}